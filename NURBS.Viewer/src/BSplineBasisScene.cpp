#include "BSplineBasisScene.hpp"

#include "Palette.hpp"
#include "SceneWidgets.hpp"

#include <Graphics/Animation.hpp>
#include <Graphics/Sampling.hpp>
#include <Graphics/Widgets.hpp>

#include <NURBS/bspline_basis.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <initializer_list>
#include <iterator>
#include <utility>

namespace NURBS::Viewer
{

namespace
{

constexpr int kMaxDegree = 6;

constexpr ImU32 kCursorColor = IM_COL32(200, 200, 210, 160);
// The knot span u0 lies in.
constexpr ImU32 kSpanColor = IM_COL32(255, 255, 255, 22);
// Outside the domain, where an unclamped basis does not sum to one.
constexpr ImU32 kOutsideColor = IM_COL32(0, 0, 0, 70);

// The knot rail below the graphs, with coincident knots stacked downwards.
constexpr double kRailY = -0.08;
constexpr double kStackStep = 0.05;
// Within this many pixels of another knot, a dragged knot snaps onto it.
constexpr double kSnapPixels = 6.0;

// The partition-of-unity bar, just right of the knots, as shares of their range.
constexpr double kBarGap = 0.08;
constexpr double kBarWidth = 0.06;

// Half the length of a tangent at u0, as a share of the domain.
constexpr double kTangentHalfWidth = 0.06;
// The highlight on the row of the graphed order in the derivatives table.
constexpr ImU32 kSelectedRowColor = IM_COL32(255, 255, 255, 30);
// The opacity of the focused function's support.
constexpr float kSupportAlpha = 0.12f;

// The p+1 copies of value.
std::vector<double> Repeated(double value, std::size_t p)
{
    return std::vector<double>(p + 1, value);
}

// {0^(p+1), interior, 1^(p+1)}.
std::vector<double> Clamped(std::size_t p, std::initializer_list<double> interior)
{
    std::vector<double> knots = Repeated(0.0, p);
    knots.insert(knots.end(), interior);
    const std::vector<double> ones = Repeated(1.0, p);
    knots.insert(knots.end(), ones.begin(), ones.end());
    return knots;
}

struct Preset
{
    const char* name;
    std::vector<double> (*knots)(std::size_t p);
};

const std::array<Preset, 5>& Presets()
{
    static const std::array<Preset, 5> presets{{
        {"Clamped uniform", [](std::size_t p) { return Clamped(p, {0.2, 0.4, 0.6, 0.8}); }},
        // As many knots as the clamped presets, evenly spaced over [0, 1].
        {"Unclamped uniform",
         [](std::size_t p)
         {
             const std::size_t count = 2 * p + 6;
             std::vector<double> knots(count);
             for (std::size_t k = 0; k < count; ++k)
                 knots[k] = Graphics::ParameterAt(0.0, 1.0, k, count);
             return knots;
         }},
        {"Clamped nonuniform", [](std::size_t p) { return Clamped(p, {0.1, 0.25, 0.6, 0.7}); }},
        // An interior knot of multiplicity p, where the basis is only C0.
        {"Repeated interior knot",
         [](std::size_t p)
         {
             std::vector<double> knots = Clamped(p, {0.25});
             const auto mid = std::ranges::find(knots, 1.0);
             knots.insert(mid, std::max<std::size_t>(p, 1), 0.5);
             knots.insert(std::ranges::find(knots, 1.0), 0.75);
             return knots;
         }},
        // No interior knots: the basis is the Bernstein basis of degree p.
        {"Bezier (= Bernstein)", [](std::size_t p) { return Clamped(p, {}); }},
    }};
    return presets;
}

// True when knots, of a degree p basis, are non-decreasing, give at least
// p+1 basis functions, have no knot of multiplicity above p+1, and end the
// domain with a nonzero span, where FindSpan puts its end.
bool IsValidKnotVector(std::size_t p, const std::vector<double>& knots)
{
    if (knots.size() < 2 * p + 2 || !std::ranges::is_sorted(knots))
        return false;

    const std::size_t n = knots.size() - p - 2;
    if (!(knots[n] < knots[n + 1]))
        return false;

    std::size_t multiplicity = 1;
    for (std::size_t k = 1; k < knots.size(); ++k)
    {
        multiplicity = knots[k] == knots[k - 1] ? multiplicity + 1 : 1;
        if (multiplicity > p + 1)
            return false;
    }
    return true;
}

// The number of knots before k equal to knots[k], so its height in a stack.
std::size_t StackLevel(const std::vector<double>& knots, std::size_t k)
{
    std::size_t level = 0;
    while (level < k && knots[k - level - 1] == knots[k])
        ++level;
    return level;
}

} // namespace

BSplineBasisScene::BSplineBasisScene()
{
    LoadPreset(0);
}

void BSplineBasisScene::LoadPreset(std::size_t preset)
{
    m_preset = preset;
    m_animateU = false;
    m_draggedKnot = -1;
    m_derivOrder = std::min(m_derivOrder, m_degree);
    m_knots = Presets()[preset].knots(static_cast<std::size_t>(m_degree));
    const glm::dvec2 domain = Domain();
    m_u0 = std::clamp(m_u0, domain.x, domain.y);
    m_focus = std::min(m_focus, static_cast<int>(LastIndex()));
    FitView();
}

bool BSplineBasisScene::SetKnots(std::vector<double> knots)
{
    if (!IsValidKnotVector(static_cast<std::size_t>(m_degree), knots))
        return false;

    m_knots = std::move(knots);
    const glm::dvec2 domain = Domain();
    m_u0 = std::clamp(m_u0, domain.x, domain.y);
    m_focus = std::min(m_focus, static_cast<int>(LastIndex()));
    return true;
}

void BSplineBasisScene::FitView()
{
    const double first = m_knots.front();
    const double last = m_knots.back();
    const double range = last - first;
    const double right = ShowsPartition() ? last + range * (kBarGap + kBarWidth + 0.05) : last + range * 0.05;
    const double bottom = RailY() - kStackStep * static_cast<double>(m_degree) - 0.05;
    m_view.FitTo({first - range * 0.05, bottom}, {right, 1.05});
}

std::size_t BSplineBasisScene::LastIndex() const
{
    return m_knots.size() - static_cast<std::size_t>(m_degree) - 2;
}

glm::dvec2 BSplineBasisScene::Domain() const
{
    return {m_knots[static_cast<std::size_t>(m_degree)], m_knots[LastIndex() + 1]};
}

double BSplineBasisScene::RailY() const
{
    // Derivatives are scaled into [-1, 1], so the rail drops below -1.
    return m_derivOrder > 0 ? kRailY - 1.0 : kRailY;
}

bool BSplineBasisScene::ShowsPartition() const
{
    return m_showPartition && m_derivOrder == 0;
}

bool BSplineBasisScene::Focuses() const
{
    return m_focus >= 0 && m_derivOrder == 0;
}

ImU32 BSplineBasisScene::ColorOf(std::size_t k, float alpha) const
{
    const float hue = static_cast<float>(k) / static_cast<float>(LastIndex() + 1);
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    ImGui::ColorConvertHSVtoRGB(hue, 0.65f, 1.0f, r, g, b);
    return ImGui::ColorConvertFloat4ToU32(ImVec4(r, g, b, alpha));
}

std::vector<std::vector<glm::dvec2>> BSplineBasisScene::SampledBasis() const
{
    const auto p = static_cast<std::size_t>(m_degree);
    const glm::dvec2 domain = Domain();

    // Even samples plus the knots themselves, so the kinks there are drawn sharp.
    const auto count = static_cast<std::size_t>(m_sampleCount);
    std::vector<double> params;
    params.reserve(count + m_knots.size());
    for (std::size_t s = 0; s < count; ++s)
        params.push_back(Graphics::ParameterAt(domain.x, domain.y, s, count));
    std::ranges::copy_if(m_knots, std::back_inserter(params),
                         [domain](double u) { return domain.x < u && u < domain.y; });
    std::ranges::sort(params);
    const auto [last, end] = std::ranges::unique(params);
    params.erase(last, end);

    // One DersBasisFuns call on span i gives the p+1 functions nonzero there,
    // and their derivatives; every other function is zero.
    const auto order = static_cast<std::size_t>(m_derivOrder);
    std::vector<std::vector<glm::dvec2>> basis(LastIndex() + 1);
    for (auto& graph : basis)
        graph.reserve(params.size() + m_knots.size());
    const auto sample = [&](std::size_t i, double u)
    {
        const std::vector<double> values = NURBS::DersBasisFuns(i, u, p, order, m_knots)[order];
        for (std::size_t k = 0; k < basis.size(); ++k)
        {
            const bool nonzero = k + p >= i && k <= i;
            basis[k].emplace_back(u, nonzero ? values[k + p - i] : 0.0);
        }
    };
    for (const double u : params)
    {
        const std::size_t i = NURBS::FindSpan(p, u, m_knots);
        // At an interior knot u_i, first the left limit, from the last
        // nonempty span ending there.
        if (domain.x < u && u == m_knots[i])
        {
            std::size_t left = i - 1;
            while (m_knots[left] == u)
                --left;
            sample(left, u);
        }
        sample(i, u);
    }
    return basis;
}

void BSplineBasisScene::DrawUI()
{
    if (PresetCombo(Presets(), m_preset))
        LoadPreset(m_preset);
    if (DegreeSlider("Degree p", m_degree, kMaxDegree))
        LoadPreset(m_preset);
    ImGui::SetItemTooltip("Changing the degree reloads the preset's knots");
    if (ImGui::SliderInt("Derivative order k", &m_derivOrder, 0, m_degree, "%d", ImGuiSliderFlags_AlwaysClamp))
        FitView();
    ImGui::SetItemTooltip("Graph the k-th derivatives N^(k)_{i,p}, from A2.3, scaled to fit");

    glm::dvec2 domain = Domain();
    if (m_animateU)
        Graphics::AdvanceWrapping(m_u0, domain.x, domain.y, SweepStep(m_sweepSeconds));
    Graphics::PlayPauseSlider("u0", "###animateU", "Animate u0: the evaluation line sweeps across the domain",
                              m_u0, domain.x, domain.y, m_animateU);
    SweepTimeSlider(m_sweepSeconds);

    ImGui::SeparatorText("Display");
    ImGui::BeginDisabled(m_derivOrder > 0);
    if (ImGui::Checkbox("Show partition of unity", &m_showPartition))
        FitView();
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Shown only for the functions themselves, at k = 0");
    ImGui::Checkbox("Show tangents at u0", &m_showTangents);
    ImGui::SetItemTooltip("The tangent to each graphed function at u0, with slope from the next derivative");
    ImGui::Checkbox("Show Cox-de Boor triangle", &m_showTriangle);
    ImGui::BeginDisabled(m_derivOrder > 0);
    const Graphics::Label16 focusPreview =
        m_focus < 0 ? Graphics::FormatLabel("None") : Graphics::IndexedLabel("N", static_cast<std::size_t>(m_focus));
    if (ImGui::BeginCombo("Focus function", focusPreview.c_str()))
    {
        if (ImGui::Selectable("None", m_focus < 0))
            m_focus = -1;
        for (std::size_t k = 0; k <= LastIndex(); ++k)
        {
            const bool selected = static_cast<int>(k) == m_focus;
            if (ImGui::Selectable(Graphics::IndexedLabel("N", k).c_str(), selected))
                m_focus = static_cast<int>(k);
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("One function N_{i,p} from A2.4, with its support; only at k = 0");

    DrawKnotEditor();

    const auto p = static_cast<std::size_t>(m_degree);
    const std::size_t span = NURBS::FindSpan(p, m_u0, m_knots);
    const std::vector<double> values = NURBS::BasisFuns(span, m_u0, p, m_knots);

    ImGui::SeparatorText("Values at u0 (A2.2)");
    ImGui::Text("FindSpan (A2.1): i = %zu,  u0 in [u%zu, u%zu)", span, span, span + 1);
    DrawValuesTable(span, values);

    if (m_showTriangle)
    {
        ImGui::SeparatorText("Cox-de Boor triangle at u0");
        DrawTriangle(span);
    }

    ImGui::SeparatorText("Derivatives at u0 (A2.3)");
    DrawDersTable(span);
    ImGui::TextWrapped("Each N_{k,p} is a degree p polynomial on a span, so derivatives above p vanish. At a knot "
                       "of multiplicity m the basis is C^(p-m): derivative p-m+1 jumps there.");

    if (Focuses())
    {
        ImGui::SeparatorText("One function at u0 (A2.4)");
        DrawOneFunction(span);
    }

    ImGui::Spacing();
    ImGui::TextWrapped("N_{k,0}(u) = 1 if u_k <= u < u_{k+1}, else 0");
    ImGui::TextWrapped("N_{k,q}(u) = (u - u_k) / (u_{k+q} - u_k) N_{k,q-1}(u)"
                       " + (u_{k+q+1} - u) / (u_{k+q+1} - u_{k+1}) N_{k+1,q-1}(u)");
    ImGui::TextWrapped("On the span [u_i, u_{i+1}) only N_{i-p,p} .. N_{i,p} are nonzero; A2.2 computes "
                       "just those p+1, sharing terms between them.");
    domain = Domain();
    ImGui::TextDisabled("m = %zu,  n = %zu,  domain [%.3f, %.3f]", m_knots.size() - 1, LastIndex(), domain.x,
                        domain.y);
}

void BSplineBasisScene::DrawKnotEditor()
{
    ImGui::SeparatorText("Knots U");
    const auto p = static_cast<std::size_t>(m_degree);

    std::size_t removed = m_knots.size();
    for (std::size_t k = 0; k < m_knots.size(); ++k)
    {
        ImGui::PushID(static_cast<int>(k));
        ImGui::SetNextItemWidth(Graphics::WidthBeside(Graphics::ButtonWidth("x")));
        double value = m_knots[k];
        if (ImGui::InputDouble("##knot", &value, 0.0, 0.0, "%.3f"))
        {
            // Kept between its neighbours, so its index stays put while typing.
            const double low = k > 0 ? m_knots[k - 1] : value;
            const double high = k + 1 < m_knots.size() ? m_knots[k + 1] : value;
            std::vector<double> knots = m_knots;
            knots[k] = std::min(std::max(value, low), high);
            SetKnots(std::move(knots));
        }
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::BeginDisabled(m_knots.size() <= 2 * p + 2);
        if (ImGui::Button("x"))
            removed = k;
        ImGui::EndDisabled();
        ImGui::SetItemTooltip("Remove u%zu", k);
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::Text("u%zu", k);
        ImGui::PopID();
    }
    if (removed < m_knots.size())
    {
        std::vector<double> knots = m_knots;
        knots.erase(knots.begin() + static_cast<std::ptrdiff_t>(removed));
        SetKnots(std::move(knots));
    }

    if (ImGui::Button("Insert knot"))
    {
        // At the middle of the widest span in the domain.
        const auto first = m_knots.begin() + m_degree;
        const auto last = m_knots.begin() + static_cast<std::ptrdiff_t>(LastIndex() + 1);
        auto widest = first;
        for (auto it = first; it != last; ++it)
        {
            if (*(it + 1) - *it > *(widest + 1) - *widest)
                widest = it;
        }
        std::vector<double> knots = m_knots;
        knots.insert(knots.begin() + std::distance(m_knots.begin(), widest) + 1, (*widest + *(widest + 1)) / 2.0);
        SetKnots(std::move(knots));
    }
    ImGui::SetItemTooltip("Insert a knot at the middle of the widest span in the domain");
    ImGui::TextDisabled("Left drag a knot below the graphs to move it; drop it on another to repeat it");
}

void BSplineBasisScene::DrawViewOptions()
{
    SampleCountSlider(m_sampleCount);
    ImGui::Spacing();
    if (Graphics::FitViewButton(Graphics::Viewport2D::kNavigationHint))
        FitView();
}

void BSplineBasisScene::DrawValuesTable(std::size_t span, const std::vector<double>& values) const
{
    if (!Graphics::BeginTableWithHeaders("values", {"j", "k = i-p+j", "N_{k,p}(u0)"}))
        return;

    const auto p = static_cast<std::size_t>(m_degree);
    double sum = 0.0;
    for (std::size_t j = 0; j <= p; ++j)
    {
        const std::size_t k = span - p + j;
        sum += values[j];
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%zu", j);
        ImGui::TableNextColumn();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ColorOf(k)), "%zu", k);
        ImGui::TableNextColumn();
        ImGui::Text("%.6f", values[j]);
    }

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("sum");
    ImGui::TableNextColumn();
    ImGui::TableNextColumn();
    ImGui::Text("%.6f", sum);
    ImGui::EndTable();
}

void BSplineBasisScene::DrawTriangle(std::size_t span) const
{
    // Row q holds N_{i-q,q} .. N_{i,q}, from BasisFuns at degree q, in the
    // columns of their index k, so each row is one wider than the one above.
    const auto p = static_cast<std::size_t>(m_degree);
    constexpr ImGuiTableFlags kFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
    if (!ImGui::BeginTable("triangle", static_cast<int>(p) + 2, kFlags))
        return;

    ImGui::TableSetupColumn("q \\ k");
    for (std::size_t k = span - p; k <= span; ++k)
        ImGui::TableSetupColumn(Graphics::FormatLabel("{}", k).c_str());
    ImGui::TableHeadersRow();

    for (std::size_t q = 0; q <= p; ++q)
    {
        const std::vector<double> values = NURBS::BasisFuns(span, m_u0, q, m_knots);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%zu", q);
        for (std::size_t c = 0; c <= p; ++c)
        {
            ImGui::TableNextColumn();
            if (c + q >= p)
                ImGui::Text("%.4f", values[c + q - p]);
        }
    }
    ImGui::EndTable();
}

void BSplineBasisScene::DrawDersTable(std::size_t span) const
{
    // Row q holds the q-th derivatives of N_{i-p,p} .. N_{i,p}, as one
    // DersBasisFuns call returns them, and their sum: 1 for the functions,
    // and 0 for every derivative, as the sum is constant.
    const auto p = static_cast<std::size_t>(m_degree);
    const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(span, m_u0, p, p, m_knots);
    constexpr ImGuiTableFlags kFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
    if (!ImGui::BeginTable("ders", static_cast<int>(p) + 3, kFlags))
        return;

    ImGui::TableSetupColumn("q \\ k");
    for (std::size_t k = span - p; k <= span; ++k)
        ImGui::TableSetupColumn(Graphics::FormatLabel("{}", k).c_str());
    ImGui::TableSetupColumn("sum");
    ImGui::TableHeadersRow();

    for (std::size_t q = 0; q <= p; ++q)
    {
        ImGui::TableNextRow();
        if (static_cast<int>(q) == m_derivOrder)
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, kSelectedRowColor);
        ImGui::TableNextColumn();
        ImGui::Text("%zu", q);
        double sum = 0.0;
        for (std::size_t j = 0; j <= p; ++j)
        {
            sum += ders[q][j];
            ImGui::TableNextColumn();
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ColorOf(span - p + j)), "%.4g", ders[q][j]);
        }
        ImGui::TableNextColumn();
        // Rounding leaves derivative sums a hair off zero; show them as 0.
        ImGui::Text("%.4g", std::abs(sum) < 1e-9 ? 0.0 : sum);
    }
    ImGui::EndTable();
}

void BSplineBasisScene::DrawOneFunction(std::size_t span) const
{
    const auto p = static_cast<std::size_t>(m_degree);
    const auto i = static_cast<std::size_t>(m_focus);
    const double value = NURBS::OneBasisFun(i, m_u0, p, m_knots);
    const bool inSupport = span >= i && span <= i + p;

    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ColorOf(i)), "N_{%zu,%zu}(u0) = %.6f", i, p, value);
    ImGui::Text("Support [u%zu, u%zu) = [%.3f, %.3f): u0 %s", i, i + p + 1, m_knots[i], m_knots[i + p + 1],
                inSupport ? "inside" : "outside, so 0 at once");
    // The same value is entry j = k - (i - p) of BasisFuns on the span.
    if (inSupport)
    {
        const std::vector<double> values = NURBS::BasisFuns(span, m_u0, p, m_knots);
        ImGui::TextDisabled("BasisFuns (A2.2) entry j = %zu: %.6f", i + p - span, values[i + p - span]);
    }
    else
    {
        ImGui::TextDisabled("Span %zu is not in [%zu, %zu], so BasisFuns (A2.2) omits it", span, i, i + p);
    }

    // Row q holds N_{i,q} .. N_{i+p-q,q} in the columns of their index k, so
    // each row is one narrower than the one above, down to N_{i,p}. They come
    // from BasisFuns at degree q, as in the Cox-de Boor triangle, zero where
    // the span is not in a function's support.
    constexpr ImGuiTableFlags kFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
    if (!ImGui::BeginTable("onebasis", static_cast<int>(p) + 2, kFlags))
        return;

    ImGui::TableSetupColumn("q \\ k");
    for (std::size_t k = i; k <= i + p; ++k)
        ImGui::TableSetupColumn(Graphics::FormatLabel("{}", k).c_str());
    ImGui::TableHeadersRow();

    for (std::size_t q = 0; q <= p; ++q)
    {
        const std::vector<double> values = NURBS::BasisFuns(span, m_u0, q, m_knots);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%zu", q);
        for (std::size_t k = i; k <= i + p - q; ++k)
        {
            ImGui::TableNextColumn();
            const bool nonzero = k + q >= span && k <= span;
            ImGui::Text("%.4f", nonzero ? values[k + q - span] : 0.0);
        }
    }
    ImGui::EndTable();

    ImGui::TextWrapped("A2.4 evaluates one N_{i,p} with no knot span, returning 0 at once outside its support; "
                       "A2.2 is cheaper when all p+1 nonzero functions are needed.");
}

void BSplineBasisScene::DrawKnotRail()
{
    std::vector<glm::dvec2> markers(m_knots.size());
    for (std::size_t k = 0; k < m_knots.size(); ++k)
        markers[k] = {m_knots[k], RailY() - kStackStep * static_cast<double>(StackLevel(m_knots, k))};

    if (m_view.DragPoints(markers, m_draggedKnot))
    {
        const auto dragged = static_cast<std::size_t>(m_draggedKnot);
        double u = markers[dragged].x;
        for (std::size_t k = 0; k < m_knots.size(); ++k)
        {
            if (k != dragged && std::abs(m_knots[k] - u) * m_view.PixelsPerUnit() < kSnapPixels)
                u = m_knots[k];
        }

        // Moved to its sorted place, which may pass other knots.
        std::vector<double> knots = m_knots;
        knots.erase(knots.begin() + m_draggedKnot);
        const auto at = std::ranges::upper_bound(knots, u);
        const std::ptrdiff_t index = std::distance(knots.begin(), at);
        knots.insert(at, u);
        if (SetKnots(std::move(knots)))
            m_draggedKnot = static_cast<int>(index);
    }

    m_view.Line({m_knots.front(), RailY()}, {m_knots.back(), RailY()}, kNeutralColor);
    for (std::size_t k = 0; k < m_knots.size(); ++k)
    {
        const std::size_t level = StackLevel(m_knots, k);
        const glm::dvec2 at{m_knots[k], RailY() - kStackStep * static_cast<double>(level)};
        m_view.Point(at, kNeutralColor, PointRadius(static_cast<int>(k) == m_draggedKnot));

        // The multiplicity, beside the bottom of each stack of two or more.
        const bool bottom = k + 1 == m_knots.size() || m_knots[k + 1] != m_knots[k];
        if (bottom && level > 0)
            m_view.Label(at, Graphics::FormatLabel("x{}", level + 1).c_str(), kNeutralColor);
    }
}

void BSplineBasisScene::DrawViewport()
{
    m_view.Begin("##bsplinebasis");

    // Edit the knots first, so everything below is drawn from the same ones.
    DrawKnotRail();

    const auto p = static_cast<std::size_t>(m_degree);
    const auto order = static_cast<std::size_t>(m_derivOrder);
    const glm::dvec2 domain = Domain();
    const std::size_t span = NURBS::FindSpan(p, m_u0, m_knots);

    // Derivatives grow with the order and the knot density, so they are
    // scaled to reach 1 at most, about a zero line.
    std::vector<std::vector<glm::dvec2>> basis = SampledBasis();
    double scale = 1.0;
    if (order > 0)
    {
        double largest = 0.0;
        for (const auto& graph : basis)
        {
            for (const glm::dvec2& point : graph)
                largest = std::max(largest, std::abs(point.y));
        }
        if (largest > 0.0)
            scale = 1.0 / largest;
        for (auto& graph : basis)
        {
            for (glm::dvec2& point : graph)
                point.y *= scale;
        }
    }
    const double graphBottom = order > 0 ? -1.0 : 0.0;

    // Outside the domain, and the span u0 lies in.
    if (m_knots.front() < domain.x)
        m_view.Rect({m_knots.front(), graphBottom}, {domain.x, 1.0}, kOutsideColor);
    if (domain.y < m_knots.back())
        m_view.Rect({domain.y, graphBottom}, {m_knots.back(), 1.0}, kOutsideColor);
    m_view.Rect({m_knots[span], graphBottom}, {m_knots[span + 1], 1.0}, kSpanColor);
    m_view.Label({m_knots[span], 1.0}, Graphics::FormatLabel("span {}", span).c_str(), kNeutralColor);
    if (order > 0)
    {
        m_view.Line({m_knots.front(), 0.0}, {m_knots.back(), 0.0}, kNeutralColor);
        char label[32];
        std::snprintf(label, sizeof(label), "N^(%zu) x %.3g", order, scale);
        m_view.Label({m_knots.front(), graphBottom}, label, kNeutralColor);
    }

    // The support [u_i, u_{i+p+1}) of the focused function.
    const auto focus = static_cast<std::size_t>(m_focus);
    if (Focuses())
    {
        m_view.Rect({m_knots[focus], 0.0}, {m_knots[focus + p + 1], 1.0}, ColorOf(focus, kSupportAlpha));
        m_view.Label({m_knots[focus], 0.9}, Graphics::FormatLabel("supp N{}", focus).c_str(), ColorOf(focus));
    }

    // Every N^(k)_{i,p}, with those vanishing on the span dimmed, or all but
    // the focused one.
    for (std::size_t k = 0; k < basis.size(); ++k)
    {
        const bool bright = Focuses() ? k == focus : k + p >= span && k <= span;
        m_view.Polyline(basis[k], ColorOf(k, bright ? 1.0f : 0.3f), bright ? 2.5f : 1.5f);
    }

    // The evaluation line at u0 and the value of every nonzero function on it,
    // with its tangent there, whose slope is the next derivative.
    m_view.Line({m_u0, graphBottom}, {m_u0, 1.0}, kCursorColor);
    const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(span, m_u0, p, order + 1, m_knots);
    const double halfWidth = (domain.y - domain.x) * kTangentHalfWidth;
    for (std::size_t j = 0; j <= p; ++j)
    {
        const std::size_t k = span - p + j;
        if (Focuses() && k != focus)
            continue;
        const glm::dvec2 at{m_u0, ders[order][j] * scale};
        if (m_showTangents)
        {
            const glm::dvec2 offset{halfWidth, ders[order + 1][j] * scale * halfWidth};
            m_view.Line(at - offset, at + offset, ColorOf(k), 1.5f);
        }
        m_view.LabeledPoint(at, Graphics::IndexedLabel("N", k).c_str(), ColorOf(k));
    }
    // Outside its support the focused function is not among those, and
    // OneBasisFun returns 0 there at once.
    if (Focuses() && !(focus + p >= span && focus <= span))
    {
        const glm::dvec2 at{m_u0, NURBS::OneBasisFun(focus, m_u0, p, m_knots)};
        m_view.LabeledPoint(at, Graphics::IndexedLabel("N", focus).c_str(), ColorOf(focus));
    }

    if (ShowsPartition())
    {
        // The values stacked bottom to top; they always reach 1.
        const double range = m_knots.back() - m_knots.front();
        const double left = m_knots.back() + range * kBarGap;
        const double right = left + range * kBarWidth;
        double bottom = 0.0;
        for (std::size_t j = 0; j <= p; ++j)
        {
            const double top = bottom + ders[0][j];
            m_view.Rect({left, bottom}, {right, top}, ColorOf(span - p + j));
            bottom = top;
        }
        m_view.Line({left, 1.0}, {right, 1.0}, kNeutralColor);
        char label[32];
        std::snprintf(label, sizeof(label), "sum = %.3f", bottom);
        m_view.Label({left, bottom}, label, kNeutralColor);
    }

    m_view.End();
}

} // namespace NURBS::Viewer
