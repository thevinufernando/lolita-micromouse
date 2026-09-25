/* main() for the PORTED build of the differential test.
 *
 * The original's Main.c still owns main(); the port's equivalent was renamed
 * FloodFill_Run() so the firmware could keep its own. This supplies the entry
 * point so the two binaries can be driven identically. The maze is carved by a
 * constructor in floodfill_sim_api.c, which both builds share. */
#include "floodfill_run.h"

int main(void)
{
    FloodFill_Run();
    return 0;
}
