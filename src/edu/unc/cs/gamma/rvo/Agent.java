/*
 * Agent.java
 * RVO2 Library Java
 *
 * Copyright (c) 2008-2016 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the Office of Technology Development at the University
 * of North Carolina at Chapel Hill <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
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
 * <http://gamma.cs.unc.edu/RVO2/>
 */

package edu.unc.cs.gamma.rvo;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.Pair;

import java.util.ArrayList;
import java.util.List;

/**
 * Defines an agent in the simulation.
 */
class Agent {
    final List<Pair<Double, Agent>> agentNeighbors = new ArrayList<>();
    final List<Pair<Double, Obstacle>> obstacleNeighbors = new ArrayList<>();
    final List<Line> lines = new ArrayList<>();
    Vector2D position = Vector2D.ZERO;
    Vector2D preferredVelocity = Vector2D.ZERO;
    Vector2D velocity = Vector2D.ZERO;
    int id = 0;
    int maxNeighbors = 0;
    double maxSpeed = 0.0;
    double neighborDistance = 0.0;
    double radius = 0.0;
    double timeHorizonAgents = 0.0;
    double timeHorizonObstacles = 0.0;

    private Vector2D newVelocity = Vector2D.ZERO;

    /**
     * Computes the neighbors of this agent.
     */
    void computeNeighbors() {
        obstacleNeighbors.clear();
        final double range = timeHorizonObstacles * maxSpeed + radius;
        Simulator.instance.kdTree.computeObstacleNeighbors(this, range * range);

        agentNeighbors.clear();

        if (maxNeighbors > 0) {
            Simulator.instance.kdTree.computeAgentNeighbors(this, neighborDistance * neighborDistance);
        }
    }

    /**
     * Computes the new velocity of this agent.
     */
    void computeNewVelocity() {
        lines.clear();

        final double invTimeHorizonObstacle = 1.0 / timeHorizonObstacles;

        // Create obstacle ORCA lines.
        for (final Pair<Double, Obstacle> obstacleNeighbor : obstacleNeighbors) {
            Obstacle obstacle1 = obstacleNeighbor.getSecond();
            Obstacle obstacle2 = obstacle1.next;

            final Vector2D relativePosition1 = obstacle1.point.subtract(position);
            final Vector2D relativePosition2 = obstacle2.point.subtract(position);

            // Check if velocity obstacle of obstacle is already taken care of
            // by previously constructed obstacle ORCA lines.
            boolean alreadyCovered = false;

            for (final Line orcaLine : lines) {
                if (RVOMath.det(relativePosition1.scalarMultiply(invTimeHorizonObstacle).subtract(orcaLine.point), orcaLine.direction) - invTimeHorizonObstacle * radius >= -RVOMath.EPSILON && RVOMath.det(relativePosition2.scalarMultiply(invTimeHorizonObstacle).subtract(orcaLine.point), orcaLine.direction) - invTimeHorizonObstacle * radius >= -RVOMath.EPSILON) {
                    alreadyCovered = true;

                    break;
                }
            }

            if (alreadyCovered) {
                continue;
            }

            // Not yet covered. Check for collisions.
            final double distanceSq1 = relativePosition1.getNormSq();
            final double distanceSq2 = relativePosition2.getNormSq();
            final double radiusSq = radius * radius;

            final Vector2D obstacleVector = obstacle2.point.subtract(obstacle1.point);
            final double s = -relativePosition1.dotProduct(obstacleVector) / obstacleVector.getNormSq();
            final double distanceSqLine = relativePosition1.add(s, obstacleVector).getNormSq();

            if (s < 0.0 && distanceSq1 <= radiusSq) {
                // Collision with left vertex. Ignore if non-convex.
                if (obstacle1.convex) {
                    final Vector2D direction = new Vector2D(-relativePosition1.getY(), relativePosition1.getX()).normalize();
                    lines.add(new Line(Vector2D.ZERO, direction));
                }

                continue;
            }

            if (s > 1.0 && distanceSq2 <= radiusSq) {
                // Collision with right vertex. Ignore if non-convex or if it
                // will be taken care of by neighboring obstacle.
                if (obstacle2.convex && RVOMath.det(relativePosition2, obstacle2.direction) >= 0.0) {
                    final Vector2D direction = new Vector2D(-relativePosition2.getY(), relativePosition2.getX()).normalize();
                    lines.add(new Line(Vector2D.ZERO, direction));
                }

                continue;
            }

            if (s >= 0.0 && s < 1.0 && distanceSqLine <= radiusSq) {
                // Collision with obstacle segment.
                final Vector2D direction = obstacle1.direction.negate();
                lines.add(new Line(Vector2D.ZERO, direction));

                continue;
            }

            // No collision. Compute legs. When obliquely viewed, both legs can
            // come from a single vertex. Legs extend cut-off line when
            // non-convex vertex.
            Vector2D leftLegDirection;
            Vector2D rightLegDirection;

            if (s < 0.0 && distanceSqLine <= radiusSq) {
                // Obstacle viewed obliquely so that left vertex defines
                // velocity obstacle.
                if (!obstacle1.convex) {
                    // Ignore obstacle.
                    continue;
                }

                obstacle2 = obstacle1;

                final double leg1 = FastMath.sqrt(distanceSq1 - radiusSq);
                leftLegDirection = new Vector2D(relativePosition1.getX() * leg1 - relativePosition1.getY() * radius, relativePosition1.getX() * radius + relativePosition1.getY() * leg1).scalarMultiply(1.0 / distanceSq1);
                rightLegDirection = new Vector2D(relativePosition1.getX() * leg1 + relativePosition1.getY() * radius, -relativePosition1.getX() * radius + relativePosition1.getY() * leg1).scalarMultiply(1.0 / distanceSq1);
            } else if (s > 1.0 && distanceSqLine <= radiusSq) {
                // Obstacle viewed obliquely so that right vertex defines
                // velocity obstacle.
                if (!obstacle2.convex) {
                    // Ignore obstacle.
                    continue;
                }

                obstacle1 = obstacle2;

                final double leg2 = FastMath.sqrt(distanceSq2 - radiusSq);
                leftLegDirection = new Vector2D(relativePosition2.getX() * leg2 - relativePosition2.getY() * radius, relativePosition2.getX() * radius + relativePosition2.getY() * leg2).scalarMultiply(1.0 / distanceSq2);
                rightLegDirection = new Vector2D(relativePosition2.getX() * leg2 + relativePosition2.getY() * radius, -relativePosition2.getX() * radius + relativePosition2.getY() * leg2).scalarMultiply(1.0 / distanceSq2);
            } else {
                // Usual situation.
                if (obstacle1.convex) {
                    final double leg1 = FastMath.sqrt(distanceSq1 - radiusSq);
                    leftLegDirection = new Vector2D(relativePosition1.getX() * leg1 - relativePosition1.getY() * radius, relativePosition1.getX() * radius + relativePosition1.getY() * leg1).scalarMultiply(1.0 / distanceSq1);
                } else {
                    // Left vertex non-convex; left leg extends cut-off line.
                    leftLegDirection = obstacle1.direction.negate();
                }

                if (obstacle2.convex) {
                    final double leg2 = FastMath.sqrt(distanceSq2 - radiusSq);
                    rightLegDirection = new Vector2D(relativePosition2.getX() * leg2 + relativePosition2.getY() * radius, -relativePosition2.getX() * radius + relativePosition2.getY() * leg2).scalarMultiply(1.0 / distanceSq2);
                } else {
                    // Right vertex non-convex; right leg extends cut-off line.
                    rightLegDirection = obstacle1.direction;
                }
            }

            // Legs can never point into neighboring edge when convex vertex,
            // take cut-off line of neighboring edge instead. If velocity
            // projected on "foreign" leg, no constraint is added.
            boolean leftLegForeign = false;
            boolean rightLegForeign = false;

            if (obstacle1.convex && RVOMath.det(leftLegDirection, obstacle1.previous.direction.negate()) >= 0.0) {
                // Left leg points into obstacle.
                leftLegDirection = obstacle1.previous.direction.negate();
                leftLegForeign = true;
            }

            if (obstacle2.convex && RVOMath.det(rightLegDirection, obstacle2.direction) <= 0.0) {
                // Right leg points into obstacle.
                rightLegDirection = obstacle2.direction;
                rightLegForeign = true;
            }

            // Compute cut-off centers.
            final Vector2D leftCutOff = obstacle1.point.subtract(position).scalarMultiply(invTimeHorizonObstacle);
            final Vector2D rightCutOff = obstacle2.point.subtract(position).scalarMultiply(invTimeHorizonObstacle);
            final Vector2D cutOffVector = rightCutOff.subtract(leftCutOff);

            // Project current velocity on velocity obstacle.

            // Check if current velocity is projected on cutoff circles.
            final double t = obstacle1 == obstacle2 ? 0.5 : velocity.subtract(leftCutOff).dotProduct(cutOffVector) / cutOffVector.getNormSq();
            final double tLeft = velocity.subtract(leftCutOff).dotProduct(leftLegDirection);
            final double tRight = velocity.subtract(rightCutOff).dotProduct(rightLegDirection);

            if (t < 0.0 && tLeft < 0.0 || obstacle1 == obstacle2 && tLeft < 0.0 && tRight < 0.0) {
                // Project on left cut-off circle.
                final Vector2D unitW = velocity.subtract(leftCutOff).normalize();

                final Vector2D direction = new Vector2D(unitW.getY(), -unitW.getX());
                final Vector2D point = leftCutOff.add(radius * invTimeHorizonObstacle, unitW);
                lines.add(new Line(point, direction));

                continue;
            }

            if (t > 1.0 && tRight < 0.0) {
                // Project on right cut-off circle.
                final Vector2D unitW = velocity.subtract(rightCutOff).normalize();

                final Vector2D direction = new Vector2D(unitW.getY(), -unitW.getX());
                final Vector2D point = rightCutOff.add(radius * invTimeHorizonObstacle, unitW);
                lines.add(new Line(point, direction));

                continue;
            }

            // Project on left leg, right leg, or cut-off line, whichever is
            // closest to velocity.
            final double distanceSqCutOff = t < 0.0 || t > 1.0 || obstacle1 == obstacle2 ? Double.POSITIVE_INFINITY : velocity.distanceSq(leftCutOff.add(cutOffVector.scalarMultiply(t)));
            final double distanceSqLeft = tLeft < 0.0 ? Double.POSITIVE_INFINITY : velocity.distanceSq(leftCutOff.add(leftLegDirection.scalarMultiply(tLeft)));
            final double distanceSqRight = tRight < 0.0 ? Double.POSITIVE_INFINITY : velocity.distanceSq(rightCutOff.add(rightLegDirection.scalarMultiply(tRight)));

            if (distanceSqCutOff <= distanceSqLeft && distanceSqCutOff <= distanceSqRight) {
                // Project on cut-off line.
                final Vector2D direction = obstacle1.direction.negate();
                final Vector2D point = leftCutOff.add(radius * invTimeHorizonObstacle, new Vector2D(-direction.getY(), direction.getX()));
                lines.add(new Line(point, direction));

                continue;
            }

            if (distanceSqLeft <= distanceSqRight) {
                // Project on left leg.
                if (leftLegForeign) {
                    continue;
                }

                final Vector2D point = leftCutOff.add(radius * invTimeHorizonObstacle, new Vector2D(-leftLegDirection.getY(), leftLegDirection.getX()));
                lines.add(new Line(point, leftLegDirection));

                continue;
            }

            // Project on right leg.
            if (rightLegForeign) {
                continue;
            }

            final Vector2D direction = rightLegDirection.negate();
            final Vector2D point = rightCutOff.add(radius * invTimeHorizonObstacle, new Vector2D(-direction.getY(), direction.getX()));
            lines.add(new Line(point, direction));
        }

        final int numObstacleLines = lines.size();

        final double invTimeHorizon = 1.0 / timeHorizonAgents;

        // Create agent ORCA lines.
        for (final Pair<Double, Agent> agentNeighbor : agentNeighbors) {
            final Agent other = agentNeighbor.getSecond();

            final Vector2D relativePosition = other.position.subtract(position);
            final Vector2D relativeVelocity = velocity.subtract(other.velocity);
            final double distanceSq = relativePosition.getNormSq();
            final double combinedRadius = radius + other.radius;
            final double combinedRadiusSq = combinedRadius * combinedRadius;

            final Vector2D direction;
            final Vector2D u;

            if (distanceSq > combinedRadiusSq) {
                // No collision.
                final Vector2D w = relativeVelocity.subtract(invTimeHorizon, relativePosition);

                // Vector from cutoff center to relative velocity.
                final double wLengthSq = w.getNormSq();
                final double dotProduct1 = w.dotProduct(relativePosition);

                if (dotProduct1 < 0.0 && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq) {
                    // Project on cut-off circle.
                    final double wLength = FastMath.sqrt(wLengthSq);
                    final Vector2D unitW = w.scalarMultiply(1.0 / wLength);

                    direction = new Vector2D(unitW.getY(), -unitW.getX());
                    u = unitW.scalarMultiply(combinedRadius * invTimeHorizon - wLength);
                } else {
                    // Project on legs.
                    final double leg = FastMath.sqrt(distanceSq - combinedRadiusSq);

                    if (RVOMath.det(relativePosition, w) > 0.0) {
                        // Project on left leg.
                        direction = new Vector2D(relativePosition.getX() * leg - relativePosition.getY() * combinedRadius, relativePosition.getX() * combinedRadius + relativePosition.getY() * leg).scalarMultiply(1.0 / distanceSq);
                    } else {
                        // Project on right leg.
                        direction = new Vector2D(relativePosition.getX() * leg + relativePosition.getY() * combinedRadius, -relativePosition.getX() * combinedRadius + relativePosition.getY() * leg).scalarMultiply(-1.0 / distanceSq);
                    }

                    final double dotProduct2 = relativeVelocity.dotProduct(direction);
                    u = direction.scalarMultiply(dotProduct2).subtract(relativeVelocity);
                }
            } else {
                // Collision. Project on cut-off circle of time timeStep.
                final double invTimeStep = 1.0 / Simulator.instance.timeStep;

                // Vector from cutoff center to relative velocity.
                final Vector2D w = relativeVelocity.subtract(invTimeStep, relativePosition);

                final double wLength = w.getNorm();
                final Vector2D unitW = w.scalarMultiply(1.0 / wLength);

                direction = new Vector2D(unitW.getY(), -unitW.getX());
                u = unitW.scalarMultiply(combinedRadius * invTimeStep - wLength);
            }

            final Vector2D point = velocity.add(0.5, u);
            lines.add(new Line(point, direction));
        }

        final int lineFail = linearProgram2(lines, preferredVelocity, false);

        if (lineFail < lines.size()) {
            linearProgram3(numObstacleLines, lineFail);
        }
    }

    /**
     * Inserts an agent neighbor into the set of neighbors of this agent.
     *
     * @param agent   A pointer to the agent to be inserted.
     * @param rangeSq The squared range around this agent.
     * @return The squared range around this agent.
     */
    double insertAgentNeighbor(Agent agent, double rangeSq) {
        if (this != agent) {
            final double distSq = position.distanceSq(agent.position);

            if (distSq < rangeSq) {
                if (agentNeighbors.size() < maxNeighbors) {
                    agentNeighbors.add(new Pair<>(distSq, agent));
                }

                int i = agentNeighbors.size() - 1;

                while (i != 0 && distSq < agentNeighbors.get(i - 1).getFirst()) {
                    agentNeighbors.set(i, agentNeighbors.get(i - 1));
                    i--;
                }

                agentNeighbors.set(i, new Pair<>(distSq, agent));

                if (agentNeighbors.size() == maxNeighbors) {
                    rangeSq = agentNeighbors.get(agentNeighbors.size() - 1).getFirst();
                }
            }
        }

        return rangeSq;
    }

    /**
     * Inserts a static obstacle neighbor into the set of neighbors of this
     * agent.
     *
     * @param obstacle The number of the static obstacle to be inserted.
     * @param rangeSq  The squared range around this agent.
     */
    void insertObstacleNeighbor(Obstacle obstacle, double rangeSq) {
        final Obstacle nextObstacle = obstacle.next;

        final double r = position.subtract(obstacle.point).dotProduct(nextObstacle.point.subtract(obstacle.point)) / nextObstacle.point.distanceSq(obstacle.point);
        final double distSq;

        if (r < 0.0) {
            distSq = position.distanceSq(obstacle.point);
        } else if (r > 1.0) {
            distSq = position.distanceSq(nextObstacle.point);
        } else {
            distSq = position.distanceSq(obstacle.point.add(nextObstacle.point.subtract(obstacle.point).scalarMultiply(r)));
        }

        if (distSq < rangeSq) {
            obstacleNeighbors.add(new Pair<>(distSq, obstacle));

            int i = obstacleNeighbors.size() - 1;

            while (i != 0 && distSq < obstacleNeighbors.get(i - 1).getFirst()) {
                obstacleNeighbors.set(i, obstacleNeighbors.get(i - 1));
                i--;
            }

            obstacleNeighbors.set(i, new Pair<>(distSq, obstacle));
        }
    }

    /**
     * Updates the two-dimensional position and two-dimensional velocity of this
     * agent.
     */
    void update() {
        velocity = newVelocity;
        position = position.add(Simulator.instance.timeStep, velocity);
    }

    /**
     * Solves a one-dimensional linear program on a specified line subject to
     * linear constraints defined by lines and a circular constraint.
     *
     * @param lines                Lines defining the linear constraints.
     * @param lineNo               The specified line constraint.
     * @param optimizationVelocity The optimization velocity.
     * @param optimizeDirection    True if the direction should be optimized.
     * @return True if successful.
     */
    private boolean linearProgram1(List<Line> lines, int lineNo, Vector2D optimizationVelocity, boolean optimizeDirection) {
        final double dotProduct = lines.get(lineNo).point.dotProduct(lines.get(lineNo).direction);
        final double discriminant = dotProduct * dotProduct + maxSpeed * maxSpeed - lines.get(lineNo).point.getNormSq();

        if (discriminant < 0.0) {
            // Max speed circle fully invalidates line lineNo.
            return false;
        }

        final double sqrtDiscriminant = FastMath.sqrt(discriminant);
        double tLeft = -sqrtDiscriminant - dotProduct;
        double tRight = sqrtDiscriminant - dotProduct;

        for (int i = 0; i < lineNo; i++) {
            final double denominator = RVOMath.det(lines.get(lineNo).direction, lines.get(i).direction);
            final double numerator = RVOMath.det(lines.get(i).direction, lines.get(lineNo).point.subtract(lines.get(i).point));

            if (FastMath.abs(denominator) <= RVOMath.EPSILON) {
                // Lines lineNo and i are (almost) parallel.
                if (numerator < 0.0) {
                    return false;
                }

                continue;
            }

            final double t = numerator / denominator;

            if (denominator >= 0.0) {
                // Line i bounds line lineNo on the right.
                tRight = FastMath.min(tRight, t);
            } else {
                // Line i bounds line lineNo on the left.
                tLeft = FastMath.max(tLeft, t);
            }

            if (tLeft > tRight) {
                return false;
            }
        }

        if (optimizeDirection) {
            // Optimize direction.
            if (optimizationVelocity.dotProduct(lines.get(lineNo).direction) > 0.0) {
                // Take right extreme.
                newVelocity = lines.get(lineNo).point.add(tRight, lines.get(lineNo).direction);
            } else {
                // Take left extreme.
                newVelocity = lines.get(lineNo).point.add(tLeft, lines.get(lineNo).direction);
            }
        } else {
            // Optimize closest point.
            final double t = lines.get(lineNo).direction.dotProduct(optimizationVelocity.subtract(lines.get(lineNo).point));

            if (t < tLeft) {
                newVelocity = lines.get(lineNo).point.add(tLeft, lines.get(lineNo).direction);
            } else if (t > tRight) {
                newVelocity = lines.get(lineNo).point.add(tRight, lines.get(lineNo).direction);
            } else {
                newVelocity = lines.get(lineNo).point.add(t, lines.get(lineNo).direction);
            }
        }

        return true;
    }

    /**
     * Solves a two-dimensional linear program subject to linear constraints
     * defined by lines and a circular constraint.
     *
     * @param lines                Lines defining the linear constraints.
     * @param optimizationVelocity The optimization velocity.
     * @param optimizeDirection    True if the direction should be optimized.
     * @return The number of the line on which it fails, or the number of lines
     * if successful.
     */
    private int linearProgram2(List<Line> lines, Vector2D optimizationVelocity, boolean optimizeDirection) {
        if (optimizeDirection) {
            // Optimize direction. Note that the optimization velocity is of unit length in this case.
            newVelocity = optimizationVelocity.scalarMultiply(maxSpeed);
        } else if (optimizationVelocity.getNormSq() > maxSpeed * maxSpeed) {
            // Optimize closest point and outside circle.
            newVelocity = optimizationVelocity.normalize().scalarMultiply(maxSpeed);
        } else {
            // Optimize closest point and inside circle.
            newVelocity = optimizationVelocity;
        }

        for (int lineNo = 0; lineNo < lines.size(); lineNo++) {
            if (RVOMath.det(lines.get(lineNo).direction, lines.get(lineNo).point.subtract(newVelocity)) > 0.0) {
                // Result does not satisfy constraint i. Compute new optimal
                // result.
                final Vector2D tempResult = newVelocity;
                if (!linearProgram1(lines, lineNo, optimizationVelocity, optimizeDirection)) {
                    newVelocity = tempResult;

                    return lineNo;
                }
            }
        }

        return lines.size();
    }

    /**
     * Solves a two-dimensional linear program subject to linear constraints
     * defined by lines and a circular constraint.
     *
     * @param numObstacleLines Count of obstacle lines.
     * @param beginLine        The line on which the 2-D linear program failed.
     */
    private void linearProgram3(int numObstacleLines, int beginLine) {
        double distance = 0.0;

        for (int i = beginLine; i < lines.size(); i++) {
            if (RVOMath.det(lines.get(i).direction, lines.get(i).point.subtract(newVelocity)) > distance) {
                // Result does not satisfy constraint of line i.
                final List<Line> projectedLines = new ArrayList<>(numObstacleLines);
                for (int j = 0; j < numObstacleLines; j++) {
                    projectedLines.add(lines.get(j));
                }

                for (int j = numObstacleLines; j < i; j++) {
                    final double determinant = RVOMath.det(lines.get(i).direction, lines.get(j).direction);
                    final Vector2D point;

                    if (FastMath.abs(determinant) <= RVOMath.EPSILON) {
                        // Line i and line j are parallel.
                        if (lines.get(i).direction.dotProduct(lines.get(j).direction) > 0.0) {
                            // Line i and line j point in the same direction.
                            continue;
                        }

                        // Line i and line j point in opposite direction.
                        point = lines.get(i).point.add(lines.get(j).point).scalarMultiply(0.5);
                    } else {
                        point = lines.get(i).point.add(lines.get(i).direction.scalarMultiply(RVOMath.det(lines.get(j).direction, lines.get(i).point.subtract(lines.get(j).point)) / determinant));
                    }

                    final Vector2D direction = lines.get(j).direction.subtract(lines.get(i).direction).normalize();
                    projectedLines.add(new Line(point, direction));
                }

                final Vector2D tempResult = newVelocity;
                if (linearProgram2(projectedLines, new Vector2D(-lines.get(i).direction.getY(), lines.get(i).direction.getX()), true) < projectedLines.size()) {
                    // This should in principle not happen. The result is by
                    // definition already in the feasible region of this linear
                    // program. If it fails, it is due to small floating point
                    // error, and the current result is kept.
                    newVelocity = tempResult;
                }

                distance = RVOMath.det(lines.get(i).direction, lines.get(i).point.subtract(newVelocity));
            }
        }
    }
}
