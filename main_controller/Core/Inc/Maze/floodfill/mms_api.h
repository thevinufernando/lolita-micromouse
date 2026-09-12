#ifndef MMS_API_H
#define MMS_API_H

/* Forget this cell's cached wall reading and any latched turn failure.
 *
 * The API layer holds one cell's worth of sensor state between calls, because
 * the algorithm asks for the same walls several times per cell. That cache is
 * only valid where the robot is standing, so anything that starts a fresh run
 * has to clear it rather than inherit whatever the last one left. */
void MMS_ApiReset(void);

#endif /* MMS_API_H */
