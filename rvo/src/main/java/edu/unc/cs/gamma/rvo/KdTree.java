/*
 * KdTree.java
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

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.Pair;

/** Defines k-D trees for agents and static obstacles in the simulation. */
class KdTree {
  /** Defines a node of an agent k-D tree. */
  private static class AgentTreeNode {
    int begin = 0;
    int end = 0;
    int left = 0;
    int right = 0;
    double maxX = 0.0;
    double maxY = 0.0;
    double minX = 0.0;
    double minY = 0.0;
  }

  /** Defines a node of an obstacle k-D tree. */
  private static class ObstacleTreeNode {
    Obstacle obstacle = null;
    ObstacleTreeNode left = null;
    ObstacleTreeNode right = null;
  }

  /** The maximum size of an agent k-D tree leaf. */
  private static final int MAX_LEAF_SIZE = 10;

  private Agent[] agents = null;
  private AgentTreeNode[] agentTree = null;
  private ObstacleTreeNode obstacleTree = null;

  /** Builds an agent k-D tree. */
  void buildAgentTree() {
    if (agents == null || agents.length != Simulator.instance.agents.size()) {
      agents = new Agent[Simulator.instance.agents.size()];

      for (int agentNo = 0; agentNo < agents.length; agentNo++) {
        agents[agentNo] = Simulator.instance.agents.get(agentNo);
      }

      agentTree = new AgentTreeNode[2 * agents.length];

      for (int nodeNo = 0; nodeNo < agentTree.length; nodeNo++) {
        agentTree[nodeNo] = new AgentTreeNode();
      }
    }

    if (agents.length != 0) {
      buildAgentTreeRecursive(0, agents.length, 0);
    }
  }

  /** Builds an obstacle k-D tree. */
  void buildObstacleTree() {
    obstacleTree = new ObstacleTreeNode();

    final List<Obstacle> obstacles = new ArrayList<>(Simulator.instance.obstacles.size());

    for (int obstacleNo = 0; obstacleNo < Simulator.instance.obstacles.size(); obstacleNo++) {
      obstacles.add(Simulator.instance.obstacles.get(obstacleNo));
    }

    obstacleTree = buildObstacleTreeRecursive(obstacles);
  }

  /**
   * Computes the agent neighbors of the specified agent.
   *
   * @param agent The agent for which agent neighbors are to be computed.
   * @param rangeSq The squared range around the agent.
   */
  void computeAgentNeighbors(Agent agent, double rangeSq) {
    queryAgentTreeRecursive(agent, rangeSq, 0);
  }

  /**
   * Computes the obstacle neighbors of the specified agent.
   *
   * @param agent The agent for which obstacle neighbors are to be computed.
   * @param rangeSq The squared range around the agent.
   */
  void computeObstacleNeighbors(Agent agent, double rangeSq) {
    queryObstacleTreeRecursive(agent, rangeSq, obstacleTree);
  }

  /**
   * Queries the visibility between two points within a specified radius.
   *
   * @param q1 The first point between which visibility is to be tested.
   * @param q2 The second point between which visibility is to be tested.
   * @param radius The radius within which visibility is to be tested.
   * @return True if q1 and q2 are mutually visible within the radius; false otherwise.
   */
  boolean queryVisibility(Vector2D q1, Vector2D q2, double radius) {
    return queryVisibilityRecursive(q1, q2, radius, obstacleTree);
  }

  /**
   * Recursive method for building an agent k-D tree.
   *
   * @param begin The beginning agent k-D tree node node index.
   * @param end The ending agent k-D tree node index.
   * @param node The current agent k-D tree node index.
   */
  private void buildAgentTreeRecursive(int begin, int end, int node) {
    agentTree[node].begin = begin;
    agentTree[node].end = end;
    agentTree[node].maxX = agents[begin].position.getX();
    agentTree[node].maxY = agents[begin].position.getY();
    agentTree[node].minX = agentTree[node].maxX;
    agentTree[node].minY = agentTree[node].maxY;

    for (int i = begin + 1; i < end; i++) {
      agentTree[node].maxX = FastMath.max(agentTree[node].maxX, agents[i].position.getX());
      agentTree[node].minX = FastMath.min(agentTree[node].minX, agents[i].position.getX());
      agentTree[node].maxY = FastMath.max(agentTree[node].maxY, agents[i].position.getY());
      agentTree[node].minY = FastMath.min(agentTree[node].minY, agents[i].position.getY());
    }

    if (end - begin > MAX_LEAF_SIZE) {
      // No leaf node.
      final boolean isVertical =
          agentTree[node].maxX - agentTree[node].minX > agentTree[node].maxY - agentTree[node].minY;
      final double splitValue =
          0.5
              * (isVertical
                  ? agentTree[node].maxX + agentTree[node].minX
                  : agentTree[node].maxY + agentTree[node].minY);

      int left = begin;
      int right = end;

      while (left < right) {
        while (left < right
            && (isVertical ? agents[left].position.getX() : agents[left].position.getY())
                < splitValue) {
          left++;
        }

        while (right > left
            && (isVertical ? agents[right - 1].position.getX() : agents[right - 1].position.getY())
                >= splitValue) {
          right--;
        }

        if (left < right) {
          final Agent tempAgent = agents[left];
          agents[left] = agents[right - 1];
          agents[right - 1] = tempAgent;
          left++;
          right--;
        }
      }

      if (left == begin) {
        left++;
      }

      agentTree[node].left = node + 1;
      agentTree[node].right = node + 2 * (left - begin);

      buildAgentTreeRecursive(begin, left, agentTree[node].left);
      buildAgentTreeRecursive(left, end, agentTree[node].right);
    }
  }

  /**
   * Recursive method for building an obstacle k-D tree.
   *
   * @param obstacles A list of obstacles.
   * @return An obstacle k-D tree node.
   */
  private static ObstacleTreeNode buildObstacleTreeRecursive(List<Obstacle> obstacles) {
    if (obstacles.isEmpty()) {
      return null;
    }

    final ObstacleTreeNode node = new ObstacleTreeNode();

    int optimalSplit = 0;
    int minLeft = obstacles.size();
    int minRight = obstacles.size();

    for (int i = 0; i < obstacles.size(); i++) {
      int leftSize = 0;
      int rightSize = 0;

      final Obstacle obstacleI1 = obstacles.get(i);
      Obstacle obstacleI2 = obstacleI1.next;

      // Compute optimal split node.
      for (int j = 0; j < obstacles.size(); j++) {
        if (i == j) {
          continue;
        }

        final Obstacle obstacleJ1 = obstacles.get(j);
        final Obstacle obstacleJ2 = obstacleJ1.next;

        final double j1LeftOfI =
            RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
        final double j2LeftOfI =
            RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

        if (j1LeftOfI >= -RVOMath.EPSILON && j2LeftOfI >= -RVOMath.EPSILON) {
          leftSize++;
        } else if (j1LeftOfI <= RVOMath.EPSILON && j2LeftOfI <= RVOMath.EPSILON) {
          rightSize++;
        } else {
          leftSize++;
          rightSize++;
        }

        final Pair<Integer, Integer> pair1 =
            new Pair<>(FastMath.max(leftSize, rightSize), FastMath.min(leftSize, rightSize));
        final Pair<Integer, Integer> pair2 =
            new Pair<>(FastMath.max(minLeft, minRight), FastMath.min(minLeft, minRight));

        if (!((pair1.getFirst() < pair2.getFirst())
            || (pair1.getFirst() <= pair2.getFirst() && pair1.getSecond() < pair2.getSecond()))) {
          break;
        }
      }

      final Pair<Integer, Integer> pair1 =
          new Pair<>(FastMath.max(leftSize, rightSize), FastMath.min(leftSize, rightSize));
      final Pair<Integer, Integer> pair2 =
          new Pair<>(FastMath.max(minLeft, minRight), FastMath.min(minLeft, minRight));

      if (pair1.getFirst() < pair2.getFirst()
          || (pair1.getFirst() <= pair2.getFirst() && pair1.getSecond() < pair2.getSecond())) {
        minLeft = leftSize;
        minRight = rightSize;
        optimalSplit = i;
      }
    }

    // Build split node.
    final List<Obstacle> leftObstacles = new ArrayList<>(minLeft);

    for (int n = 0; n < minLeft; n++) {
      leftObstacles.add(null);
    }

    final List<Obstacle> rightObstacles = new ArrayList<>(minRight);

    for (int n = 0; n < minRight; n++) {
      rightObstacles.add(null);
    }

    int leftCounter = 0;
    int rightCounter = 0;

    final Obstacle obstacleI1 = obstacles.get(optimalSplit);
    final Obstacle obstacleI2 = obstacleI1.next;

    for (int j = 0; j < obstacles.size(); j++) {
      if (optimalSplit == j) {
        continue;
      }

      final Obstacle obstacleJ1 = obstacles.get(j);
      final Obstacle obstacleJ2 = obstacleJ1.next;

      final double j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
      final double j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

      if (j1LeftOfI >= -RVOMath.EPSILON && j2LeftOfI >= -RVOMath.EPSILON) {
        leftObstacles.set(leftCounter++, obstacles.get(j));
      } else if (j1LeftOfI <= RVOMath.EPSILON && j2LeftOfI <= RVOMath.EPSILON) {
        rightObstacles.set(rightCounter++, obstacles.get(j));
      } else {
        // Split obstacle j.
        final double t =
            RVOMath.det(
                    obstacleI2.point.subtract(obstacleI1.point),
                    obstacleJ1.point.subtract(obstacleI1.point))
                / RVOMath.det(
                    obstacleI2.point.subtract(obstacleI1.point),
                    obstacleJ1.point.subtract(obstacleJ2.point));

        final Vector2D splitPoint =
            obstacleJ1.point.add(obstacleJ2.point.subtract(obstacleJ1.point).scalarMultiply(t));

        final Obstacle newObstacle = new Obstacle();
        newObstacle.point = splitPoint;
        newObstacle.previous = obstacleJ1;
        newObstacle.next = obstacleJ2;
        newObstacle.convex = true;
        newObstacle.direction = obstacleJ1.direction;

        newObstacle.id = Simulator.instance.obstacles.size();

        Simulator.instance.obstacles.add(newObstacle);

        obstacleJ1.next = newObstacle;
        obstacleJ2.previous = newObstacle;

        if (j1LeftOfI > 0.0) {
          leftObstacles.set(leftCounter++, obstacleJ1);
          rightObstacles.set(rightCounter++, newObstacle);
        } else {
          rightObstacles.set(rightCounter++, obstacleJ1);
          leftObstacles.set(leftCounter++, newObstacle);
        }
      }
    }

    node.obstacle = obstacleI1;
    node.left = buildObstacleTreeRecursive(leftObstacles);
    node.right = buildObstacleTreeRecursive(rightObstacles);

    return node;
  }

  private static double sqr(double d) {
    return d * d;
  }

  /**
   * Recursive method for computing the agent neighbors of the specified agent.
   *
   * @param agent The agent for which agent neighbors are to be computed.
   * @param rangeSq The squared range around the agent.
   * @param node The current agent k-D tree node index.
   * @return The squared range around the agent.
   */
  private double queryAgentTreeRecursive(Agent agent, double rangeSq, int node) {
    if (agentTree[node].end - agentTree[node].begin <= MAX_LEAF_SIZE) {
      for (int agentNo = agentTree[node].begin; agentNo < agentTree[node].end; agentNo++) {
        rangeSq = agent.insertAgentNeighbor(agents[agentNo], rangeSq);
      }
    } else {
      final double distanceSqLeft =
          sqr(FastMath.max(0.0, agentTree[agentTree[node].left].minX - agent.position.getX()))
              + sqr(FastMath.max(0.0, agent.position.getX() - agentTree[agentTree[node].left].maxX))
              + sqr(FastMath.max(0.0, agentTree[agentTree[node].left].minY - agent.position.getY()))
              + sqr(
                  FastMath.max(0.0, agent.position.getY() - agentTree[agentTree[node].left].maxY));
      final double distanceSqRight =
          sqr(FastMath.max(0.0, agentTree[agentTree[node].right].minX - agent.position.getX()))
              + sqr(
                  FastMath.max(0.0, agent.position.getX() - agentTree[agentTree[node].right].maxX))
              + sqr(
                  FastMath.max(0.0, agentTree[agentTree[node].right].minY - agent.position.getY()))
              + sqr(
                  FastMath.max(0.0, agent.position.getY() - agentTree[agentTree[node].right].maxY));

      if (distanceSqLeft < distanceSqRight) {
        if (distanceSqLeft < rangeSq) {
          rangeSq = queryAgentTreeRecursive(agent, rangeSq, agentTree[node].left);

          if (distanceSqRight < rangeSq) {
            rangeSq = queryAgentTreeRecursive(agent, rangeSq, agentTree[node].right);
          }
        }
      } else {
        if (distanceSqRight < rangeSq) {
          rangeSq = queryAgentTreeRecursive(agent, rangeSq, agentTree[node].right);

          if (distanceSqLeft < rangeSq) {
            rangeSq = queryAgentTreeRecursive(agent, rangeSq, agentTree[node].left);
          }
        }
      }
    }

    return rangeSq;
  }

  /**
   * Recursive method for computing the obstacle neighbors of the specified agent.
   *
   * @param agent The agent for which obstacle neighbors are to be computed.
   * @param rangeSq The squared range around the agent.
   * @param node The current obstacle k-D node.
   */
  private static void queryObstacleTreeRecursive(
      Agent agent, double rangeSq, ObstacleTreeNode node) {
    if (node != null) {
      final Obstacle obstacle1 = node.obstacle;
      final Obstacle obstacle2 = obstacle1.next;

      final double agentLeftOfLine =
          RVOMath.leftOf(obstacle1.point, obstacle2.point, agent.position);

      queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0 ? node.left : node.right);

      final double distanceSqLine =
          agentLeftOfLine * agentLeftOfLine / obstacle2.point.distanceSq(obstacle1.point);

      if (distanceSqLine < rangeSq) {
        if (agentLeftOfLine < 0.0) {
          // Try obstacle at this node only if agent is on right side
          // of obstacle (and can see obstacle).
          agent.insertObstacleNeighbor(node.obstacle, rangeSq);
        }

        // Try other side of line.
        queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0 ? node.right : node.left);
      }
    }
  }

  /**
   * Recursive method for querying the visibility between two points within a specified radius.
   *
   * @param q1 The first point between which visibility is to be tested.
   * @param q2 The second point between which visibility is to be tested.
   * @param radius The radius within which visibility is to be tested.
   * @param node The current obstacle k-D node.
   * @return True if q1 and q2 are mutually visible within the radius; false otherwise.
   */
  private static boolean queryVisibilityRecursive(
      Vector2D q1, Vector2D q2, double radius, ObstacleTreeNode node) {
    if (node == null) {
      return true;
    }

    final Obstacle obstacle1 = node.obstacle;
    final Obstacle obstacle2 = obstacle1.next;

    final double q1LeftOfI = RVOMath.leftOf(obstacle1.point, obstacle2.point, q1);
    final double q2LeftOfI = RVOMath.leftOf(obstacle1.point, obstacle2.point, q2);
    final double invLengthI = 1.0 / obstacle2.point.distanceSq(obstacle1.point);

    if (q1LeftOfI >= 0.0 && q2LeftOfI >= 0.0) {
      return queryVisibilityRecursive(q1, q2, radius, node.left)
          && ((q1LeftOfI * q1LeftOfI * invLengthI >= radius * radius
                  && q2LeftOfI * q2LeftOfI * invLengthI >= radius * radius)
              || queryVisibilityRecursive(q1, q2, radius, node.right));
    }

    if (q1LeftOfI <= 0.0 && q2LeftOfI <= 0.0) {
      return queryVisibilityRecursive(q1, q2, radius, node.right)
          && ((q1LeftOfI * q1LeftOfI * invLengthI >= radius * radius
                  && q2LeftOfI * q2LeftOfI * invLengthI >= radius * radius)
              || queryVisibilityRecursive(q1, q2, radius, node.left));
    }

    if (q1LeftOfI >= 0.0 && q2LeftOfI <= 0.0) {
      // One can see through obstacle from left to right.
      return queryVisibilityRecursive(q1, q2, radius, node.left)
          && queryVisibilityRecursive(q1, q2, radius, node.right);
    }

    final double point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point);
    final double point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point);
    final double invLengthQ = 1.0 / q2.distanceSq(q1);

    return point1LeftOfQ * point2LeftOfQ >= 0.0
        && point1LeftOfQ * point1LeftOfQ * invLengthQ > radius * radius
        && point2LeftOfQ * point2LeftOfQ * invLengthQ > radius * radius
        && queryVisibilityRecursive(q1, q2, radius, node.left)
        && queryVisibilityRecursive(q1, q2, radius, node.right);
  }
}
