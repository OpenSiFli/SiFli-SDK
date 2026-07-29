/* Additional defines and initialisation for NetWare platform */

#include <stdio.h>
#include <nwnamspc.h>
#include <nwfileio.h>
#include <nwthread.h>
#include <nwadv.h>

int nwinit(void);

int nwinit(void) {
    NWCCODE ccode;

    /* import UnAugmentAsterisk dynamically for NW4.x compatibility */
    unsigned int myHandle = GetNLMHandle();
    void (*pUnAugmentAsterisk)(int) = (void(*)(int))
            ImportSymbol(myHandle, "UnAugmentAsterisk");
    if (pUnAugmentAsterisk)
            pUnAugmentAsterisk(1);
    UnimportSymbol(myHandle, "UnAugmentAsterisk");

    /* set long name space */
    if ((ccode = SetCurrentNameSpace(NW_NS_LONG))) {
        printf("\nSetCurrentNameSpace returned %04X\n",ccode);
        return ccode;
    }
    if ((ccode = SetTargetNameSpace(NW_NS_LONG))) {
        printf("\nSetTargetNameSpace returned %04X\n",ccode);
        return ccode;
    }
    UseAccurateCaseForPaths(1);

    printf("\r");
    return 0;
}
