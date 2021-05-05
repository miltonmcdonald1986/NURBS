#include "pch.h"
#include "CppUnitTest.h"

#include "HelloWorld.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace NURBS 
{
namespace UnitTest 
{
namespace HelloWorld 
{


TEST_CLASS(HelloWorld)
{
public:
	
	TEST_METHOD(HelloWorld_ReturnsHelloWorld)
	{
		Assert::AreEqual(NURBS::HelloWorld::HelloWorld(), std::string("Hello, world!"));
	}

};


}
}
}
