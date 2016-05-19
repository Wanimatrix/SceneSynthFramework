#include "gtest/gtest.h"
#include "input/ConfParser.h"


TEST(ConfigurationParserTest, ParserResultsCorrectConfigClass)
{
    Configuration synthesis = ConfigurationParser::parse("../config/test/synthesis.conf");
    Configuration experiment = ConfigurationParser::parse("../config/test/experiment.conf");
    EXPECT_EQ(true, cp.test());
};
