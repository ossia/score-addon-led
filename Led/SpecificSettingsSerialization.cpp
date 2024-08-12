#include "SpecificSettings.hpp"

#include <score/serialization/DataStreamVisitor.hpp>
#include <score/serialization/JSONVisitor.hpp>

template <>
void DataStreamReader::read(const Led::SpecificSettings& n)
{
  m_stream << n.device << n.num_pixels << n.speed << n.format;
  insertDelimiter();
}

template <>
void DataStreamWriter::write(Led::SpecificSettings& n)
{
  m_stream >> n.device >> n.num_pixels >> n.speed >> n.format;
  checkDelimiter();
}

template <>
void JSONReader::read(const Led::SpecificSettings& n)
{
  obj["Device"] = n.device;
  obj["Pixels"] = n.num_pixels;
  obj["Speed"] = n.speed;
  obj["Format"] = n.format;
}

template <>
void JSONWriter::write(Led::SpecificSettings& n)
{
  n.device <<= obj["Device"];
  n.num_pixels <<= obj["Pixels"];
  n.speed <<= obj["Speed"];
  n.format <<= obj["Format"];
}
