#ifndef CDCE913_H_
#define CDCE913_H_

// Settings set for the CDCE(L)913
typedef struct {
  double mul;	// Frequency multiplication given by this setting

  // The rest are implementation-specific and ca be changed if some
  // other PLL with different settings are used.
  uint16_t N;
  uint8_t P;
  uint8_t pdiv;
  uint8_t Q;
  uint8_t R;
  uint16_t M;
  uint8_t trim;
} CDCE913_PLL_Setting_t;

#define PLL_MIN_TRIM 4
#define PLL_MAX_TRIM 20
#define PLL_CENTER_TRIM ((PLL_MIN_TRIM+PLL_MAX_TRIM)/2)

extern const int16_t PLL_XTAL_TRIM_PP10M[];

// This is no typedef, as we want it to be representable as an uint8_t to thread though implementation-agnostic code.
typedef enum {
	CDCE913_OutputMode_SHUTDOWN,
	CDCE913_OutputMode_OUTPUT_1_PLL,
	CDCE913_OutputMode_OUTPUT_1_BYPASS,
	CDCE913_OutputMode_OUTPUT_12,
	CDCE913_OutputMode_OUTPUT_13,
	CDCE913_OutputMode_XO_PASSTHROUGH
} CDCE913_OutputMode_t;

#define PLL_XTAL_NOMINAL_FREQUENCY 26E6

uint8_t CDCE913_read(uint8_t);

#endif /* CDCE913_H_ */
