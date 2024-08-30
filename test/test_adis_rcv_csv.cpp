#include <adis_rcv_csv.h>
#include <gtest/gtest.h>

class AdisRcvCsvTest : public testing::Test
{
protected:
  AdisRcvCsv::Product GetPd(AdisRcvCsv& arc)
  {
    return arc.pd_;
  }
  void CheckProductId(AdisRcvCsv& arc, const std::string& id)
  {
    arc.CheckProductId(id);
  }

  virtual void SetUp()
  {
    //  arc = AdisRcvCsv();
  }
  virtual void TearDown() {}
};

TEST_F(AdisRcvCsvTest, CheckProductIdTest)
{
  AdisRcvCsv arc;
  CheckProductId(arc, "ADIS16470");
  EXPECT_EQ(GetPd(arc), AdisRcvCsv::Product::ADIS16470);

  CheckProductId(arc, "ADIS16500");
  EXPECT_EQ(GetPd(arc), AdisRcvCsv::Product::ADIS16500);

  CheckProductId(arc, "ADIS16505-2");
  EXPECT_EQ(GetPd(arc), AdisRcvCsv::Product::ADIS16505_2);

  CheckProductId(arc, "ADIS16475-2");
  EXPECT_EQ(GetPd(arc), AdisRcvCsv::Product::ADIS16475_2);

  CheckProductId(arc, "ADIS16477-2");
  EXPECT_EQ(GetPd(arc), AdisRcvCsv::Product::ADIS16477_2);

  CheckProductId(arc, "ADIS16495-2");
  EXPECT_EQ(GetPd(arc), AdisRcvCsv::Product::ADIS16495_2);

  CheckProductId(arc, "UNKNOWN_ID");
  EXPECT_EQ(GetPd(arc), AdisRcvCsv::Product::UNKNOWN);
}
