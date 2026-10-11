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
    m_knots = Presets()[preset].knots(static_cast<std::size_t>(m_degree));
    const glm::dvec2 domain = Domain();
    m_u0 = std::clamp(m_u0, domain.x, domain.y);
    FitView();
}

bool BSplineBasisScene::SetKnots(std::vector<double> knots)
{
    if (!IsValidKnotVector(static_cast<std::size_t>(m_degree), knots))
        return false;

    m_knots = std::move(knots);
    const glm::dvec2 domain = Domain();
    m_u0 = std::clamp(m_u0, domain.x, domain.y);
    return true;
}

void BSplineBasisScene::FitView()
{
    const double first = m_knots.front();
    const double last = m_knots.back();
    const double range = last - first;
    const double right = m_showPartition ? last + range * (kBarGap + kBarWidth + 0.05) : last + range * 0.05;
    const double bottom = kRailY - kStackStep * static_cast<double>(m_degree) - 0.05;
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

    // One FindSpan and BasisFuns call per sample gives the p+1 nonzero
    // functions there; every other function is zero.
    std::vector<std::vector<glm::dvec2>> basis(LastIndex() + 1);
    for (auto& graph : basis)
        graph.reserve(params.size());
    for (const double u : params)
    {
        const std::size_t i = NURBS::FindSpan(p, u, m_knots);
        const std::vector<double> values = NURBS::BasisFuns(i, u, p, m_knots);
        for (std::size_t k = 0; k < basis.size(); ++k)
        {
            const bool nonzero = k + p >= i && k <= i;
            basis[k].emplace_back(u, nonzero ? values[k + p - i] : 0.0);
        }
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

    glm::dvec2 domain = Domain();
    if (m_animateU)
        Graphics::AdvanceWrapping(m_u0, domain.x, domain.y, SweepStep(m_sweepSeconds));
    Graphics::PlayPauseSlider("u0", "###animateU", "Animate u0: the evaluation line sweeps across the domain",
                              m_u0, domain.x, domain.y, m_animateU);
    SweepTimeSlider(m_sweepSeconds);

    ImGui::SeparatorText("Display");
    if (ImGui::Checkbox("Show partition of unity", &m_showPartition))
        FitView();
    ImGui::Checkbox("Show Cox-de Boor triangle", &m_showTriangle);

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

void BSplineBasisScene::DrawKnotRail()
{
    std::vector<glm::dvec2> markers(m_knots.size());
    for (std::size_t k = 0; k < m_knots.size(); ++k)
        markers[k] = {m_knots[k], kRailY - kStackStep * static_cast<double>(StackLevel(m_knots, k))};

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

    m_view.Line({m_knots.front(), kRailY}, {m_knots.back(), kRailY}, kNeutralColor);
    for (std::size_t k = 0; k < m_knots.size(); ++k)
    {
        const std::size_t level = StackLevel(m_knots, k);
        const glm::dvec2 at{m_knots[k], kRailY - kStackStep * static_cast<double>(level)};
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
    const glm::dvec2 domain = Domain();
    const std::size_t span = NURBS::FindSpan(p, m_u0, m_knots);

    // Outside the domain, and the span u0 lies in.
    if (m_knots.front() < domain.x)
        m_view.Rect({m_knots.front(), 0.0}, {domain.x, 1.0}, kOutsideColor);
    if (domain.y < m_knots.back())
        m_view.Rect({domain.y, 0.0}, {m_knots.back(), 1.0}, kOutsideColor);
    m_view.Rect({m_knots[span], 0.0}, {m_knots[span + 1], 1.0}, kSpanColor);
    m_view.Label({m_knots[span], 1.0}, Graphics::FormatLabel("span {}", span).c_str(), kNeutralColor);

    // Every N_{k,p}, with those vanishing on the span dimmed.
    const std::vector<std::vector<glm::dvec2>> basis = SampledBasis();
    for (std::size_t k = 0; k < basis.size(); ++k)
    {
        const bool nonzero = k + p >= span && k <= span;
        m_view.Polyline(basis[k], ColorOf(k, nonzero ? 1.0f : 0.3f), nonzero ? 2.5f : 1.5f);
    }

    // The evaluation line at u0 and the value of every nonzero function on it.
    m_view.Line({m_u0, 0.0}, {m_u0, 1.0}, kCursorColor);
    const std::vector<double> values = NURBS::BasisFuns(span, m_u0, p, m_knots);
    for (std::size_t j = 0; j <= p; ++j)
    {
        const std::size_t k = span - p + j;
        m_view.LabeledPoint({m_u0, values[j]}, Graphics::IndexedLabel("N", k).c_str(), ColorOf(k));
    }

    if (m_showPartition)
    {
        // The values stacked bottom to top; they always reach 1.
        const double range = m_knots.back() - m_knots.front();
        const double left = m_knots.back() + range * kBarGap;
        const double right = left + range * kBarWidth;
        double bottom = 0.0;
        for (std::size_t j = 0; j <= p; ++j)
        {
            const double top = bottom + values[j];
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
