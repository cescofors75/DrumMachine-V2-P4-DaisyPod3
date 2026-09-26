#include "../../shared/fx_rotary_selection.h"
#include <cassert>
#include <cstdio>
int main() {
    FxRotarySelection select;
    assert(select.matchesRow(0));
    assert(select.touch(9)); // column 2, row 3
    assert(select.cells[0]==0 && select.cells[1]==9 && select.cells[2]==2 && select.cells[3]==3);
    assert(select.touch(15));
    assert(select.cells[1]==9 && select.cells[3]==15);
    assert(!select.matchesRow(0));
    select.row(2); assert(select.matchesRow(2));
    select.row(4); assert(select.cells[0]==16 && select.cells[1]==17 && select.cells[2]==-1 && select.cells[3]==-1);
    assert(select.touch(2)); assert(select.cells[2]==2 && select.cells[0]==16);
    assert(!select.touch(18)); assert(!select.touch(-1));
    for(int row=0;row<5;++row) { select.row(row); assert(select.matchesRow(row)); }
    std::puts("FX rotary selection: mixed rows, fixed columns, whole-row replacement, empty slots PASS");
}
