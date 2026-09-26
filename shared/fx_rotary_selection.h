#pragma once
struct FxRotarySelection {
    int cells[4]={0,1,2,3};
    bool touch(int cell) {
        if(cell<0 || cell>=18 || cells[cell%4]==cell) return false;
        cells[cell%4]=cell; return true;
    }
    void row(int r) {
        if(r<0 || r>=5) return;
        for(int i=0;i<4;++i) cells[i]=r*4+i<18 ? r*4+i : -1;
    }
    bool matchesRow(int r) const {
        for(int i=0;i<4;++i) if(cells[i]!=(r*4+i<18 ? r*4+i : -1)) return false;
        return true;
    }
};
