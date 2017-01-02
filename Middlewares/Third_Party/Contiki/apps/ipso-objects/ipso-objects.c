/*
 * Copyright (c) 2015, Yanzi Networks AB.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \addtogroup oma-lwm2m
 * @{
 */

/**
 * \file
 *         Implementation of the IPSO Objects
 * \author
 *         Joakim Eriksson <joakime@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

/*
 * 2016-09-xx Modified by ST Central Lab: added new Objects to be initialized
 */

#include "contiki.h"
#include "ipso-objects.h"
/*---------------------------------------------------------------------------*/
void
ipso_objects_init(void)
{
  /* initialize any relevant object for the IPSO Objects */
#ifdef IPSO_TEMPERATURE
  ipso_temperature_init();
#endif

#ifdef IPSO_HUMIDITY
  ipso_humidity_init();
#endif

#ifdef IPSO_BAROMETER
  ipso_barometer_init();
#endif

#ifdef IPSO_PRESENCE
  ipso_presence_init();
#endif

#ifdef IPSO_ACCELERATION
  ipso_acceleration_init();
#endif

#ifdef IPSO_MAGNETO
  ipso_magneto_init();
#endif

#if PLATFORM_HAS_BUTTON
  ipso_button_init();
#endif

#if IPSO_DIGITAL_OUTPUT
  ipso_digital_output_init();
#endif

#ifdef IPSO_LIGHT_CONTROL
  ipso_light_control_init();
#elif PLATFORM_HAS_LEDS
  ipso_leds_control_init();
#endif
}
/*---------------------------------------------------------------------------*/
/** @} */
