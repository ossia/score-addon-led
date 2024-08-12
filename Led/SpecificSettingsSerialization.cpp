#include "SpecificSettings.hpp"

#include <score/serialization/DataStreamVisitor.hpp>
#include <score/serialization/JSONVisitor.hpp>

template <>
void DataStreamReader::read(const Led::SpecificSettings& n)
{
  m_stream << n.control;
  insertDelimiter();
}

template <>
void DataStreamWriter::write(Led::SpecificSettings& n)
{
  m_stream >> n.control;
  checkDelimiter();
}

template <>
void JSONReader::read(const Led::SpecificSettings& n)
{
  obj["Control"] = n.control;
}

template <>
void JSONWriter::write(Led::SpecificSettings& n)
{
  n.control <<= obj["Control"];
}