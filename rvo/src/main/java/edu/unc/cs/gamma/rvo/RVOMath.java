/*
 * RVOMath.java
 * RVO2 Library Java
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/RVO2/>
 */

package edu.unc.cs.gamma.rvo;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

/**
 * Contains functions and constants used in multiple classes.
 */
final class RVOMath {
    /**
     * A sufficiently small positive number.
     */
    static final double EPSILON = 0.00001;

    /**
     * Computes the determinant of a two-dimensional square matrix with rows
     * consisting of the specified two-dimensional vectors.
     *
     * @param vector1 The top row of the two-dimensional square matrix.
     * @param vector2 The bottom row of the two-dimensional square matrix.
     * @return The determinant of the two-dimensional square matrix.
     */
    static double det(Vector2D vector1, Vector2D vector2) {
        return vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();
    }

    /**
     * Computes the signed distance from a line connecting the specified points
     * to a specified point.
     *
     * @param point1 The first point on the line.
     * @param point2 The second point on the line.
     * @param point3 The point to which the signed distance is to be calculated.
     * @return Positive when the point3 lies to the left of the line passing
     * through point1 and point2.
     */
    static double leftOf(Vector2D point1, Vector2D point2, Vector2D point3) {
        return det(point1.subtract(point3), point2.subtract(point1));
    }

   /**
     * Constructs and initializes an instance.
     */
    private RVOMath() {
    }
}
