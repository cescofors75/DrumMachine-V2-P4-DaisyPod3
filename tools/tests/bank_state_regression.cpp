#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>
// Exercise the production wire observer, including packed/aligned payloads.
#include "../../P4/src/ui/bank_state.cpp"
using namespace bank_state;
int main() {
    for(int velocity=0;velocity<=127;++velocity)
        assert(hitVelocity(velocity,100,false,80,false)==velocity);
    assert(hitVelocity(80,40,true,80,false)==40);
    assert(hitVelocity(40,40,true,80,false)==20); // Preserve relative 16-level dynamics.
    assert(hitVelocity(127,127,true,32,true)==127);
    assert(hitVelocity(0,127,true,0,true)==0);
    assert(hitVelocity(20,1,true,127,false)==1);
    uint8_t data[16]={3,1};
    float cut=2000, q=10.35f, drive=.5f;
    memcpy(data+4,&cut,4); memcpy(data+8,&q,4);
    for(int size=0;size<16;++size) observe(CMD_TRACK_FILTER,data,size);
    assert(fxDirty[3]==0);
    observe(CMD_TRACK_FILTER,data,16);
    assert(fxValues[3][0]==1 && fxValues[3][1]==85 && fxValues[3][2]==64);
    fxDirty[3]=0;
    memcpy(data+4,&drive,4);
    for(int size=0;size<8;++size) observe(CMD_TRACK_DISTORTION,data,size);
    assert(fxDirty[3]==0);
    observe(CMD_TRACK_DISTORTION,data,8); assert(fxValues[3][3]==64);
    drive=std::numeric_limits<float>::max(); memcpy(data+4,&drive,4);
    observe(CMD_TRACK_DISTORTION,data,8); assert(fxValues[3][3]==127);
    drive=std::numeric_limits<float>::quiet_NaN(); memcpy(data+4,&drive,4);
    observe(CMD_TRACK_DISTORTION,data,8); assert(fxValues[3][3]==127);
    // Quantized controls must round-trip every exposed detent without getting stuck.
    for(int percent=0;percent<=100;++percent) {
        data[1]=percent; observe(CMD_TRACK_REVERB_SEND,data,2);
        observe(CMD_TRACK_DELAY_SEND,data,2);
        for(int field:{5,6}) assert(int(lroundf(fxValues[3][field]*100.f/127.f))==percent);
    }
    for(int bits=4;bits<=16;++bits) {
        data[1]=bits; observe(CMD_TRACK_BITCRUSH,data,2);
        assert(int(lroundf(16-fxValues[3][4]*12.f/127.f))==bits);
    }
    data[1]=uint8_t(int8_t(-73)); observe(CMD_TRACK_PAN,data,2); assert(pads[3].pan==-73);
    int16_t cents=-1700; memcpy(data+1,&cents,2);
    observe(CMD_PAD_PITCH,data,3); assert(pads[3].pitch==-17);
    data[1]=213; observe(CMD_PAD_FADE_IN,data,2); observe(CMD_PAD_FADE_OUT,data,2);
    assert(pads[3].fadeIn==213 && pads[3].fadeOut==213);
    data[1]=40; observe(CMD_TRACK_CHORUS_SEND,data,2); assert(pads[3].mod==40);
    observe(CMD_TRACK_CLEAR_FX,data,1);
    assert(pads[3].mod==0);
    for(int i=0;i<7;++i) assert(fxValues[3][i]==(i==1 ? 127 : 0));
    fxDirty[3]=0;
    observe(CMD_TRACK_FILTER,nullptr,16);
    data[0]=16; observe(CMD_TRACK_FILTER,data,16); assert(fxDirty[3]==0);
    param(3,0,1200); assert(synth[0][0]==1200 && synthDirty[0]==1);
    synthDirty[0]=0;
    param(2,0,5); param(3,32,5); param(3,0,std::numeric_limits<float>::quiet_NaN());
    assert(synthDirty[0]==0 && synth[0][0]==1200);
    puts("BANK STATE: packed payloads, truncation, non-finite values, pan/pitch signs, FX reset, all send/bitcrush detents PASS");
}
