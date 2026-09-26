#pragma once
#include <stdint.h>
namespace seqgroups {
constexpr uint8_t evolveBars[8]={1,2,3,4,6,8,12,16};
constexpr const char* evolveModes[9]={"OFF","MIX","PROBABILIDAD","TIMING","VELOCIDAD","GHOST","SPARSE","DENSO","SUAVE"};
constexpr const char* evolveScopes[5]={"TODOS","BOMBO","CAJA / CLAP","HI-HATS","PERCUSION"};
constexpr uint16_t masks[4]={0x0001,0x0022,0x000c,0xffd0};
constexpr const char* names[4]={"BOMBO","CAJA / CLAP","HI-HATS","PERCUSION"};
constexpr const char* variations[16]={"ORIGINAL","DESPLAZAR","ESPEJO","HALF TIME","GHOST","RATCHET","SPARSE","SHIFT +2","SHIFT +4","SHIFT +8","SWAP PARES","EUCLID 3","EUCLID 5","EUCLID 7","FILL FINAL","ACENTOS"};
constexpr const char* glitches[12]={"ORIGINAL","RATCHET x2","RATCHET x4","REV BLOQUES","CHOP","RATCHET x3","STUTTER","FILL x2","OFFBEATS","REV TOTAL","SWAP MITADES","DROP 25%"};
constexpr const char* orders[12]={"ORIGINAL","ROTAR +1","ROTAR +2","ROTAR +3","INVERTIR","SWAP 1-2","SWAP 3-4","SWAP 1-4","SWAP 2-3","ZIGZAG","PARES","CRUZADO"};
constexpr uint8_t orderMap[12][4]={{0,1,2,3},{3,0,1,2},{2,3,0,1},{1,2,3,0},{3,2,1,0},{1,0,2,3},{0,1,3,2},{3,1,2,0},{0,2,1,3},{0,3,1,2},{1,0,3,2},{2,0,3,1}};
struct Step { bool active; uint8_t velocity, probability, ratchet; };
inline bool equal(const Step& a,const Step& b) {
    return a.active==b.active && a.velocity==b.velocity && a.probability==b.probability && a.ratchet==b.ratchet;
}
inline void transform(const Step* base,Step* out,unsigned variant) {
    for(unsigned s=0;s<16;++s) {
        unsigned from=variant==1 ? (s+15)%16 : variant==2 ? 15-s : variant==3 ? s/2 : variant==13 ? (s/4)*4+3-s%4 : s;
        out[s]=base[from];
        if(variant==3 && (s%2)!=0) out[s].active=false;
        if(variant==4 && !out[s].active && s%4==3 && base[(s+1)%16].active)
            out[s]={true,42,75,1};
        if(variant==5 && out[s].active) out[s].ratchet=2;
        if(variant==6 && (s%4)!=0) out[s].active=false;
        if(variant==3 && out[s].active) out[s].ratchet=1;
        if(variant==11 && out[s].active) out[s].ratchet=2;
        if(variant==12 && out[s].active) out[s].ratchet=4;
        if(variant==14 && s%2) out[s].active=false;
    }
}
inline void variation(const Step* base,Step* out,unsigned kind) {
    if(kind<7) { transform(base,out,kind); return; }
    for(unsigned s=0;s<16;++s) {
        const unsigned from=kind==7 ? (s+14)%16 : kind==8 ? (s+12)%16 : kind==9 ? (s+8)%16 : kind==10 ? (s^1) : s;
        out[s]=base[from];
        if(kind>=11 && kind<=13) {
            const unsigned pulses=kind==11 ? 3 : kind==12 ? 5 : 7;
            out[s].active=(s*pulses)%16<pulses;
            if(out[s].active && out[s].velocity<20) out[s].velocity=75;
            out[s].probability=100; out[s].ratchet=1;
        }
        if(kind==14 && s>=12) out[s]={true,uint8_t(65+(s-12)*12),100,1};
        if(kind==15 && out[s].active) out[s].velocity=s%4==0 ? 115 : 65;
    }
}
inline void glitch(const Step* base,Step* out,unsigned kind) {
    if(kind<5) { transform(base,out,10+kind); return; }
    for(unsigned s=0;s<16;++s) {
        const unsigned from=kind==6 ? (s/2)*2 : kind==9 ? 15-s : kind==10 ? (s+8)%16 : s;
        out[s]=base[from];
        if(kind==5 && out[s].active) out[s].ratchet=3;
        if(kind==7 && s%4==3) out[s]={true,65,80,2};
        if(kind==8 && !(s%2)) out[s].active=false;
        if(kind==11 && s%4==3) out[s].active=false;
    }
}
}
