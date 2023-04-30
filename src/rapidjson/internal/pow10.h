// Tencent is pleased to support the open source community by making RapidJSON available.
// 
// Copyright (C) 2015 THL A29 Limited, a Tencent company, and Milo Yip. All rights reserved.
//
// Licensed under the MIT License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// http://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software distributed 
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR 
// CONDITIONS OF ANY KIND, either express or implied. See the License for the 
// specific language governing permissions and limitations under the License.

#ifndef RAPIDJSON_POW10_
#define RAPIDJSON_POW10_

#include "../rapidjson.h"

RAPIDJSON_NAMESPACE_BEGIN
namespace internal {

//! Computes integer powers of 10 in double (10.0^n).
/*! This function uses lookup table for fast and accurate results.
    \param n non-negative exponent. Must <= 308.
    \return 10.0^n
*/
inline double Pow10(int n) {
    static const double e[] = { // 1e-0...1e308: 309 * 8 bytes = 2472 bytes
        1e+0l,  
        1e+1l,  1e+2l,  1e+3l,  1e+4l,  1e+5l,  1e+6l,  1e+7l,  1e+8l,  1e+9l,  1e+10l, 1e+11l, 1e+12l, 1e+13l, 1e+14l, 1e+15l, 1e+16l, 1e+17l, 1e+18l, 1e+19l, 1e+20l, 
        1e+21l, 1e+22l, 1e+23l, 1e+24l, 1e+25l, 1e+26l, 1e+27l, 1e+28l, 1e+29l, 1e+30l, 1e+31l, 1e+32l, 1e+33l, 1e+34l, 1e+35l, 1e+36l, 1e+37l, 1e+38l, 1e+39l, 1e+40l,
        1e+41l, 1e+42l, 1e+43l, 1e+44l, 1e+45l, 1e+46l, 1e+47l, 1e+48l, 1e+49l, 1e+50l, 1e+51l, 1e+52l, 1e+53l, 1e+54l, 1e+55l, 1e+56l, 1e+57l, 1e+58l, 1e+59l, 1e+60l,
        1e+61l, 1e+62l, 1e+63l, 1e+64l, 1e+65l, 1e+66l, 1e+67l, 1e+68l, 1e+69l, 1e+70l, 1e+71l, 1e+72l, 1e+73l, 1e+74l, 1e+75l, 1e+76l, 1e+77l, 1e+78l, 1e+79l, 1e+80l,
        1e+81l, 1e+82l, 1e+83l, 1e+84l, 1e+85l, 1e+86l, 1e+87l, 1e+88l, 1e+89l, 1e+90l, 1e+91l, 1e+92l, 1e+93l, 1e+94l, 1e+95l, 1e+96l, 1e+97l, 1e+98l, 1e+99l, 1e+100l,
        1e+101l,1e+102l,1e+103l,1e+104l,1e+105l,1e+106l,1e+107l,1e+108l,1e+109l,1e+110l,1e+111l,1e+112l,1e+113l,1e+114l,1e+115l,1e+116l,1e+117l,1e+118l,1e+119l,1e+120l,
        1e+121l,1e+122l,1e+123l,1e+124l,1e+125l,1e+126l,1e+127l,1e+128l,1e+129l,1e+130l,1e+131l,1e+132l,1e+133l,1e+134l,1e+135l,1e+136l,1e+137l,1e+138l,1e+139l,1e+140l,
        1e+141l,1e+142l,1e+143l,1e+144l,1e+145l,1e+146l,1e+147l,1e+148l,1e+149l,1e+150l,1e+151l,1e+152l,1e+153l,1e+154l,1e+155l,1e+156l,1e+157l,1e+158l,1e+159l,1e+160l,
        1e+161l,1e+162l,1e+163l,1e+164l,1e+165l,1e+166l,1e+167l,1e+168l,1e+169l,1e+170l,1e+171l,1e+172l,1e+173l,1e+174l,1e+175l,1e+176l,1e+177l,1e+178l,1e+179l,1e+180l,
        1e+181l,1e+182l,1e+183l,1e+184l,1e+185l,1e+186l,1e+187l,1e+188l,1e+189l,1e+190l,1e+191l,1e+192l,1e+193l,1e+194l,1e+195l,1e+196l,1e+197l,1e+198l,1e+199l,1e+200l,
        1e+201l,1e+202l,1e+203l,1e+204l,1e+205l,1e+206l,1e+207l,1e+208l,1e+209l,1e+210l,1e+211l,1e+212l,1e+213l,1e+214l,1e+215l,1e+216l,1e+217l,1e+218l,1e+219l,1e+220l,
        1e+221l,1e+222l,1e+223l,1e+224l,1e+225l,1e+226l,1e+227l,1e+228l,1e+229l,1e+230l,1e+231l,1e+232l,1e+233l,1e+234l,1e+235l,1e+236l,1e+237l,1e+238l,1e+239l,1e+240l,
        1e+241l,1e+242l,1e+243l,1e+244l,1e+245l,1e+246l,1e+247l,1e+248l,1e+249l,1e+250l,1e+251l,1e+252l,1e+253l,1e+254l,1e+255l,1e+256l,1e+257l,1e+258l,1e+259l,1e+260l,
        1e+261l,1e+262l,1e+263l,1e+264l,1e+265l,1e+266l,1e+267l,1e+268l,1e+269l,1e+270l,1e+271l,1e+272l,1e+273l,1e+274l,1e+275l,1e+276l,1e+277l,1e+278l,1e+279l,1e+280l,
        1e+281l,1e+282l,1e+283l,1e+284l,1e+285l,1e+286l,1e+287l,1e+288l,1e+289l,1e+290l,1e+291l,1e+292l,1e+293l,1e+294l,1e+295l,1e+296l,1e+297l,1e+298l,1e+299l,1e+300l,
        1e+301l,1e+302l,1e+303l,1e+304l,1e+305l,1e+306l,1e+307l,1e+308l
    };
    RAPIDJSON_ASSERT(n >= 0 && n <= 308);
    return e[n];
}

} // namespace internal
RAPIDJSON_NAMESPACE_END

#endif // RAPIDJSON_POW10_
