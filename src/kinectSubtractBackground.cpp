/*
 * kinectSubtractBackground.cpp
 *
 *  Created on: Jul 31, 2013
 *      Author: dale
 */

/*
 * Algorithm:
 * 1. Subtract the background in the RGB to get ROI
 * 2. Get the Z in the ROI from the depth image
 * 3. Compute the X and Y from of each point in the ROI with info: u,v, K, D, Z
 * 4. Test by rviz
 */

/*
 * Steps:
 * 1. Write a node to advertise the foreground mask, obj
 * 2. Wrtie a node to read mask, obj and depth image to execute the Algorithm above
 */



