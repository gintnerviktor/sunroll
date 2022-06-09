/*
 *
 */

#ifndef _RELAYS_H_
#define _RELAYS_H_

/**
 * @brief initialize the relays lowlevel module
 *
 * @param none
 *
 * @return none
 */
void relays_init(void);

/**
 * @brief deinitialize the relays's lowlevel module
 *
 * @param none
 *
 * @return none
 */
void relays_deinit(void);

/**
 * @brief turn on/off the lowlevel relayOpen
 *
 * @param value The "On" value
 *
 * @return none
 */
int relayOpen_set_on(bool value);

/**
 * @brief turn on/off the lowlevel relayClose
 *
 * @param value The "On" value
 *
 * @return none
 */
int relayClose_set_on(bool value);

/**
 * @brief get relayOpen status
 * @return true if relay on, else false
 */
bool get_relayOpen_status();

/**
 * @brief get relayClose status
 * @return true if relay on, else false
 */
bool get_relayClose_status();

#endif /* _RELAYS_H_ */