#include "IRremote.h"
#include "IRremoteInt.h"

/*============================================================================*/
/*                                                                            */
/*                           #####                                            */
/*                   #####  #     #   #####  ######  #####                    */
/*                   #    # #           #    #       #    #                   */
/*                   #    #  #####      #    #####   #    #                   */
/*                   #####        #     #    #       #####                    */
/*                   #   #  #     #     #    #       #                        */
/*                   #    #  #####      #    ######  #                        */
/*                                                                            */
/*============================================================================*/

/*
 * rStep (*r*uwido *st*andard *e*ngineering *p*rotocol) is a protocol developed
 * by ruwido (https://www.ruwido.com/) and used for their customizeable remote
 * controls if no specific protocol is requested. It features a Customer ID,
 * an Address (ie. one customer may have a number of selectable keycodes on
 * one RC, so you can switch between those.)  It supports different frame types
 * (ie. keyboard (for alphanumeric keyboards), mouse, RC and an error frame)
 * as well as an (unused) battery-okay indicator.
 *
 *   This driver will use the Customer ID, Address, Frametype and Battery
 * Status bits to make up IRremote's address field. That way, a user can
 * distinguish between all frame types etc. when receiving data. Error frames
 * are dropped. It is built to support both timings supported by rStep: 38kHz
 * and 56kHz.
 */

// This is one frame, along with its interpretation:
///
// ~~~~~~______|~~~~~~______|~~~~~~______|______~~~~~~|~~~~~~______|______~~~~~~|~~~~~~______|~~~~~~______|______~~~~~~|~~~~~~______|______~~~~~~|______~~~~~~|______~~~~~~|~~~~~~______|~~~~~~______|~~~~~~______|~~~~~~______|______~~~~~~
//
//	1            1            1            0            1            0            1            1            0            1            0            0            0            1            1            1            1            0
//
//   STA=1       |            Cust=1101                              |  Addr=01                |  Frametype=10           |  Bat=1     |   Data = 00011110


//+=============================================================================
//
#if SEND_RSTEP
#error "Not yet implemented."
#endif /* SEND_RSTEP  */

//+=============================================================================
//
#if DECODE_RSTEP

static bool decodeRstepInternal (decode_results *results,
                                 unsigned int short_min, unsigned int short_max,
                                 unsigned int long_min,  unsigned int long_max);

#define RSTEP_SHORT_PULSE_38k		315	/* µsec, burst 200..460µsec, gap 160..430µsec  */
#define RSTEP_LONG_PULSE_38k		630	/* µsec, burst 520..780µsec, gap 470..750µsec  */
#define RSTEP_SHORT_PULSE_56k		213	/* µsec, burst 140..320µsec, gap 100..290µsec  */
#define RSTEP_LONG_PULSE_56k		426	/* µsec, burst 350..540µsec, gap 320..500µsec  */

#define RSTEP_SHORT_PULSE_38k_MIN_TICKS	4	/* 50µsec ticks  */
#define RSTEP_SHORT_PULSE_38k_MAX_TICKS	10	/* 50µsec ticks  */
#define RSTEP_LONG_PULSE_38k_MIN_TICKS	11	/* 50µsec ticks  */
#define RSTEP_LONG_PULSE_38k_MAX_TICKS	16	/* 50µsec ticks  */

#define RSTEP_SHORT_PULSE_56k_MIN_TICKS	2	/* 50µsec ticks  */
#define RSTEP_SHORT_PULSE_56k_MAX_TICKS	6	/* 50µsec ticks  */
#define RSTEP_LONG_PULSE_56k_MIN_TICKS	7	/* 50µsec ticks  */
#define RSTEP_LONG_PULSE_56k_MAX_TICKS	11	/* 50µsec ticks  */

bool IRrecv::decodeRstep (decode_results *results) {
	/* 38kHz is more common, try that first. If that was unsuccessful,
	   try 56kHz, which is less common, but around in the field.  */
	if (decodeRstepInternal (results,
	                         RSTEP_SHORT_PULSE_38k_MIN_TICKS, RSTEP_SHORT_PULSE_38k_MAX_TICKS,
	                         RSTEP_LONG_PULSE_38k_MIN_TICKS,  RSTEP_LONG_PULSE_38k_MAX_TICKS)
	    || decodeRstepInternal (results,
	                            RSTEP_SHORT_PULSE_56k_MIN_TICKS, RSTEP_SHORT_PULSE_56k_MAX_TICKS,
	                            RSTEP_LONG_PULSE_56k_MIN_TICKS,  RSTEP_LONG_PULSE_56k_MAX_TICKS))
		return true;

	return false;
}

static bool
decodeRstepInternal (decode_results *results,
                     unsigned int short_min_ticks, unsigned int short_max_ticks,
	             unsigned int long_min_ticks,  unsigned int long_max_ticks)
{
	uint64_t real_biphase_bits = 0;	/* Bi-phase bits separated to individual time-based bits.  */
	uint64_t real_data_bits = 0;	/* Data bits after bi-phase dissection.  */
	int num_real_biphase_bits = 0;
	int num_real_data_bits = 0;

#if DEBUG
	char buf[10];

	for (int i = 1; i < results->rawlen; i++)
		for (unsigned int ii = 0; ii < results->rawbuf[i]; ii++)
			DBG_PRINT ((i%2)? "~": "_");
	DBG_PRINTLN ("");

	for (int i = 1; i < results->rawlen; i++) {
		sprintf (buf, "%i", results->rawbuf[i]);
		DBG_PRINT (buf);
		for (unsigned int ii = 0; ii < results->rawbuf[i] - strlen (buf); ii++)
			DBG_PRINT (" ");
	}
	DBG_PRINTLN ("");
#endif /* DEBUG  */

	/* Part I: Cut those short and long MARKs and SPACEs into individual
	   bits, each representing the state in one unit of time.  */
	for (int i = 1; i < results->rawlen; i++) {
		if (i % 2 == 1)		{	/* Uneven bit number: MARK bit.  */
			if (results->rawbuf[i] >= short_min_ticks
			    && results->rawbuf[i] <= short_max_ticks) {
				real_biphase_bits |= 1ULL << num_real_biphase_bits++;
			} else if (results->rawbuf[i] >= long_min_ticks
			           && results->rawbuf[i] <= long_max_ticks) {
				real_biphase_bits |= 1ULL << num_real_biphase_bits++;
				real_biphase_bits |= 1ULL << num_real_biphase_bits++;
			} else {
				DBG_PRINT ("rawbuf[");
				DBG_PRINT (i);
				DBG_PRINTLN ("] seems to not be a mark of proper length.");
				return false;
			}
		} else { /* if (i % 2 == 0) */		/* Even rawbuf: SPACE bit.  */
			if (results->rawbuf[i] >= short_min_ticks
			    && results->rawbuf[i] <= short_max_ticks) {
				real_biphase_bits |= 0ULL << num_real_biphase_bits++;
			} else if (results->rawbuf[i] >= long_min_ticks
			           && results->rawbuf[i] <= long_max_ticks) {
				real_biphase_bits |= 0ULL << num_real_biphase_bits++;
				real_biphase_bits |= 0ULL << num_real_biphase_bits++;
			} else {
				DBG_PRINT ("rawbuf[");
				DBG_PRINT (i);
				DBG_PRINTLN ("] seems to not be a space of proper length.");
				return false;
			}
		}
	}

	/* Part II: If the bit count is uneven and ends in a MARK, we didn't
	   see the SPACE, so simply add 1 to the bit count. Note that the bit
	   mask is zero-initialized and thus contains a proper SPACE value.  */
	if (num_real_biphase_bits % 2)
		num_real_biphase_bits++;

#if DEBUG
	DBG_PRINT ("Bi-phase bits (");
	DBG_PRINT (num_real_biphase_bits);
	DBG_PRINT ("): ");
	for (int i = 0; i < num_real_biphase_bits; i++) {
		DBG_PRINT ((real_biphase_bits & (1ULL << i))? 1: 0);
		if (i % 2)
			DBG_PRINT (" ");
	}
	DBG_PRINTLN ("");
#endif /* DEBUG  */

	/* Part III: See if we have a rising or falling edge between two
	   bi-phase bits to get the actual data bits.  */
	for (int i = 0; i < num_real_biphase_bits; i += 2) {
		int lower_bit = !! (real_biphase_bits & (1ULL << (i + 0)));
		int higher_bit = !! (real_biphase_bits & (1ULL << (i + 1)));

		if (lower_bit == higher_bit) {
			DBG_PRINT ("Lower bit == higher bit at biphase bits ");
			DBG_PRINT (i);
			DBG_PRINT (" and ");
			DBG_PRINTLN (i + 1);
			return false;
		}

		if (lower_bit)	/* MARK -> SPACE ==> 1  */
			real_data_bits |= 1ULL << num_real_data_bits++;
		else		/* SPACE -> MARK ==> 0  */
			real_data_bits |= 0ULL << num_real_data_bits++;
	}

#if DEBUG
	DBG_PRINT ("Real data bits (");
	DBG_PRINT (num_real_data_bits);
	DBG_PRINT ("): ");
	for (int i = 0; i < num_real_data_bits; i++) {
		if (i == 0)
			DBG_PRINT ("Sta: ");
		if (i == 1)
			DBG_PRINT ("  Cust: ");
		if (i == 5)
			DBG_PRINT ("  Addr: ");
		if (i == 7)
			DBG_PRINT ("  FrameType: ");
		if (i == 9)
			DBG_PRINT ("  Bat: ");
		if (i == 10)
			DBG_PRINT ("  Data: ");
		DBG_PRINT ((real_data_bits & (1ULL << i))? 1: 0);
	}
	DBG_PRINTLN ("");
#endif /* DEBUG  */

	if (num_real_data_bits < 10) {
		DBG_PRINT (num_real_data_bits);
		DBG_PRINTLN (" is not enough data bits, at least 10");
		return false;
	}

	/* Copy our decoded result to the return buffer.  */
	results->decode_type = RSTEP;
	results->bits = num_real_data_bits - 10;
	/* Bits are sent in big-endian, so high-bits first, we need to shift around... */
	results->value = 0;
	for (int i = 10; i < num_real_data_bits; i++) {	/* Everything after Battery-Full indicator.  */
		results->value <<= 1;
		results->value |= !! (real_data_bits & (1ULL << i));
	}
	results->address = 0;
	for (int i = 1; i < 10; i++) {	/* Customer ID, Address, Frametype, Battery.  */
		results->address <<= 1;
		results->address |= !! (real_data_bits & (1ULL << i));
	}

	/* We've got a final positive result.  */
	DBG_PRINT   ("Bits: ");
	DBG_PRINTLN (results->bits);
	DBG_PRINT   ("Address: 0x");
	DBG_PRINTLN (results->address, HEX);
	DBG_PRINT   ("Value: 0x");
	DBG_PRINTLN (results->value, HEX);

	return true;
}

#endif /* DECODE_RSTEP  */
