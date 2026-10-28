#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

// Standard MAVLink TUNNEL, custom payload_type 32768, broadcast destination.
// Four-byte magic VLOS followed by little-endian IEEE754 float32 values.
// Version 1: configuration kind=1 (15), state kind=2 (25), environment kind=3 (12).
// Environment version 2 retains CAS at slot 4; deprecated TAS slot 3 is NaN.
namespace los4_wire {
constexpr uint16_t payload_type=32768;
inline std::array<uint8_t,128> encode(const std::vector<float>& values) {
    std::array<uint8_t,128> bytes{};
    bytes[0]='V';bytes[1]='L';bytes[2]='O';bytes[3]='S';
    if(values.size()>31)return {};
    for(std::size_t i=0;i<values.size();++i) {
        uint32_t bits;std::memcpy(&bits,&values[i],4);
        for(int j=0;j<4;++j)bytes[4+4*i+j]=static_cast<uint8_t>(bits>>(8*j));
    }
    return bytes;
}
inline std::vector<float> decode(const uint8_t* bytes,std::size_t length) {
    if(length<12 || length>128 || (length-4)%4 || bytes[0]!='V' || bytes[1]!='L' || bytes[2]!='O' || bytes[3]!='S')return {};
    std::vector<float> values((length-4)/4);
    for(std::size_t i=0;i<values.size();++i) {
        uint32_t bits=0;for(int j=0;j<4;++j)bits|=uint32_t(bytes[4+4*i+j])<<(8*j);
        std::memcpy(&values[i],&bits,4);
    }
    if((values[0]!=1 && !(values[0]==2 && values[1]==3)) || (values[1]!=1 && values[1]!=2 && values[1]!=3) ||
       (values[1]==1 && values.size()!=15) || (values[1]==2 && values.size()!=25) || (values[1]==3 && values.size()!=12))return {};
    return values;
}
}
