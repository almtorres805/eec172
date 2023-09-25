/*
 * test.h
 *
 *  Created on: Apr 17, 2023
 *      Author: almto
 */

#ifndef TEST_H_
#define TEST_H_

void delay(unsigned long ulCount);
void testfastlines(unsigned int color1, unsigned int color2);
void testdrawrects(unsigned int color);
void testfillrects(unsigned int color1, unsigned int color2);
void testfillcircles(unsigned char radius, unsigned int color);
void testlines(unsigned int color);
void testroundrects();
void testtriangles();
void lcdTestPattern2(void);
void lcdTestPattern(void);

#endif /* TEST_H_ */
