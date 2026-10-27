#include "j_platform.h"
/* Foreground radio polling never touches the shared sensor I2C bus. */
void RC_Analy(void) { JPlatform_RadioPoll(); }
