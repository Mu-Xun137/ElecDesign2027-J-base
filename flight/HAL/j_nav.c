#include "j_nav.h"
#include <string.h>
/* Base build: manual horizontal alignment. Upgrade supplies active navigation. */
void JNav_Init(JNav *n) { memset(n, 0, sizeof(*n)); }
uint8_t JNav_Start(JNav *n, const JNavInput *in) { (void)n; (void)in; return 0; }
void JNav_Step(JNav *n, const JNavInput *in) { (void)n; (void)in; }
void JNav_HoverFinished(JNav *n, uint8_t success, const JNavInput *in)
{ (void)n; (void)success; (void)in; }
uint8_t JNav_Active(const JNav *n) { (void)n; return 0; }
