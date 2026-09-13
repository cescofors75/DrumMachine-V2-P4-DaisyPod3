#include "bank_state.h"
#include "../master/protocol.h"
#include <string.h>
#include <algorithm>
namespace bank_state {
static int u7(float f) { return int(lroundf(std::max(0.f,std::min(127.f,f)))); }
void observe(uint8_t command,const void* data,uint16_t size) {
    if(!data || size<1) return;
    const auto* p=static_cast<const uint8_t*>(data); const uint8_t pad=p[0];
    if(pad>=16) return;
    switch(command) {
        case CMD_TRACK_PAN: if(size>=2) pads[pad].pan=int8_t(p[1]); break;
        case CMD_PAD_PITCH: if(size>=3) { int16_t cents; memcpy(&cents,p+1,2); pads[pad].pitch=cents/100; } break;
        case CMD_PAD_FADE_IN: if(size>=2) pads[pad].fadeIn=p[1]; break;
        case CMD_PAD_FADE_OUT: if(size>=2) pads[pad].fadeOut=p[1]; break;
        case CMD_TRACK_CHORUS_SEND: if(size>=2) pads[pad].mod=p[1]; break;
        case CMD_TRACK_REVERB_SEND: if(size>=2) fx(pad,5,u7(p[1]*127.f/100.f)); break;
        case CMD_TRACK_DELAY_SEND: if(size>=2) fx(pad,6,u7(p[1]*127.f/100.f)); break;
        case CMD_TRACK_BITCRUSH: if(size>=2) fx(pad,4,u7((16-int(p[1]))*127.f/12.f)); break;
        case CMD_TRACK_DISTORTION: if(size>=8) { float v; memcpy(&v,p+4,4); if(isfinite(v)) fx(pad,3,u7(v*127)); } break;
        case CMD_TRACK_FILTER: if(size>=16) {
            float cut,q; memcpy(&cut,p+4,4); memcpy(&q,p+8,4);
            fx(pad,0,p[1]);
            if(isfinite(cut) && cut>0) fx(pad,1,u7(logf(cut/20.f)/logf(1000.f)*127));
            if(isfinite(q)) fx(pad,2,u7((q-.7f)/19.3f*127));
        } break;
        case CMD_TRACK_CLEAR_FILTER: fx(pad,0,0); break;
        case CMD_TRACK_CLEAR_FX:
            for(uint8_t i=0;i<7;++i) fx(pad,i,i==1 ? 127 : 0);
            pads[pad].mod=0; break;
        default: break;
    }
}
}
