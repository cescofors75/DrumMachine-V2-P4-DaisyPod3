#pragma once
namespace rotary {
inline int detents(int diff,int gain=51) {
    const int magnitude=diff<0 ? -diff : diff;
    if(gain<=0 || magnitude<gain/2) return 0;
    return (diff<0 ? -1 : 1)*((magnitude+gain/2)/gain);
}
}
