/*
 * Simulator.java
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

import java.util.ArrayList;
import java.util.List;

/**
 * Defines the simulation.
 */
public class Simulator {
    /**
     * The single instance of the simulation.
     */
    public static final Simulator instance = new Simulator();

    final List<Agent> agents = new ArrayList<>();
    final List<Obstacle> obstacles = new ArrayList<>();
    final KdTree kdTree = new KdTree();
    double timeStep = 0.0;

    private Agent defaultAgent = null;
    private double globalTime = 0.0;

    /**
     * Adds a new agent with default properties to the simulation.
     *
     * @param position The two-dimensional starting position of this agent.
     * @return The number of the agent, or -1 when the agent defaults have not
     * been set.
     */
    public int addAgent(Vector2D position) {
        if (defaultAgent == null) {
            return -1;
        }

        final Agent agent = new Agent();
        agent.id = agents.size();
        agent.maxNeighbors = defaultAgent.maxNeighbors;
        agent.maxSpeed = defaultAgent.maxSpeed;
        agent.neighborDistance = defaultAgent.neighborDistance;
        agent.position = position;
        agent.radius = defaultAgent.radius;
        agent.timeHorizonAgents = defaultAgent.timeHorizonAgents;
        agent.timeHorizonObstacles = defaultAgent.timeHorizonObstacles;
        agent.velocity = defaultAgent.velocity;
        agents.add(agent);

        return agent.id;
    }

    /**
     * Adds a new agent to the simulation.
     *
     * @param position             The two-dimensional starting position of this
     *                             agent.
     * @param neighborDistance     The maximum distance (center point to center
     *                             point) to other agents this agent takes into
     *                             account in the navigation. The larger this
     *                             number, the longer the running time of the
     *                             simulation. If the number is too low, the
     *                             simulation will not be safe. Must be
     *                             non-negative.
     * @param maxNeighbors         The maximum number of other agents this agent
     *                             takes into account in the navigation. The
     *                             larger this number, the longer the running
     *                             time of the simulation. If the number is too
     *                             low, the simulation will not be safe.
     * @param timeHorizonAgents    The minimal amount of time for which this
     *                             agent's velocities that are computed by the
     *                             simulation are safe with respect to other
     *                             agents. The larger this number, the sooner
     *                             this agent will respond to the presence of
     *                             other agents, but the less freedom this agent
     *                             has in choosing its velocities. Must be
     *                             positive.
     * @param timeHorizonObstacles The minimal amount of time for which this
     *                             agent's velocities that are computed by the
     *                             simulation are safe with respect to
     *                             obstacles. The larger this number, the sooner
     *                             this agent will respond to the presence of
     *                             obstacles, but the less freedom this agent
     *                             has in choosing its velocities. Must be
     *                             positive.
     * @param radius               The radius of this agent. Must be
     *                             non-negative.
     * @param maxSpeed             The maximum speed of this agent. Must be
     *                             non-negative.
     * @param velocity             The initial two-dimensional linear velocity
     *                             of this agent.
     * @return The number of the agent.
     */
    public int addAgent(Vector2D position, double neighborDistance, int maxNeighbors, double timeHorizonAgents, double timeHorizonObstacles, double radius, double maxSpeed, Vector2D velocity) {
        final Agent agent = new Agent();
        agent.id = agents.size();
        agent.maxNeighbors = maxNeighbors;
        agent.maxSpeed = maxSpeed;
        agent.neighborDistance = neighborDistance;
        agent.position = position;
        agent.radius = radius;
        agent.timeHorizonAgents = timeHorizonAgents;
        agent.timeHorizonObstacles = timeHorizonObstacles;
        agent.velocity = velocity;
        agents.add(agent);

        return agent.id;
    }

    /**
     * Adds a new obstacle to the simulation. To add a "negative" obstacle,
     * e.g., a bounding polygon around the environment, the vertices should be
     * listed in clockwise order.
     *
     * @param vertices ArrayList of the vertices of the polygonal obstacle in
     *                 counterclockwise order.
     * @return The number of the first vertex of the obstacle, or -1 when the
     * number of vertices is less than two.
     */
    public int addObstacle(List<Vector2D> vertices) {
        if (vertices.size() < 2) {
            return -1;
        }

        final int obstacleNo = obstacles.size();

        for (int vertexNo = 0; vertexNo < vertices.size(); vertexNo++) {
            final Obstacle obstacle = new Obstacle();
            obstacle.point = vertices.get(vertexNo);

            if (vertexNo != 0) {
                obstacle.previous = obstacles.get(obstacles.size() - 1);
                obstacle.previous.next = obstacle;
            }

            if (vertexNo == vertices.size() - 1) {
                obstacle.next = obstacles.get(obstacleNo);
                obstacle.next.previous = obstacle;
            }

            obstacle.direction = vertices.get(vertexNo == vertices.size() - 1 ? 0 : vertexNo + 1).subtract(vertices.get(vertexNo)).normalize();

            obstacle.convex = vertices.size() == 2 || RVOMath.leftOf(vertices.get(vertexNo == 0 ? vertices.size() - 1 : vertexNo - 1), vertices.get(vertexNo), vertices.get(vertexNo == vertices.size() - 1 ? 0 : vertexNo + 1)) >= 0.0;

            obstacle.id = obstacles.size();
            obstacles.add(obstacle);
        }

        return obstacleNo;
    }

    /**
     * Performs a simulation step and updates the two-dimensional position and
     * two-dimensional velocity of each agent.
     *
     * @return The global time after the simulation step.
     */
    public double doStep() {
        kdTree.buildAgentTree();

        for (final Agent agent : agents) {
            agent.computeNeighbors();
            agent.computeNewVelocity();
        }

        agents.forEach(Agent::update);

        globalTime += timeStep;

        return globalTime;
    }

    /**
     * Returns the specified agent neighbor of the specified agent.
     *
     * @param agentNo    The number of the agent whose agent neighbor is to be
     *                   retrieved.
     * @param neighborNo The number of the agent neighbor to be retrieved.
     * @return The number of the neighboring agent.
     */
    public int getAgentAgentNeighbor(int agentNo, int neighborNo) {
        return agents.get(agentNo).agentNeighbors.get(neighborNo).getSecond().id;
    }

    /**
     * Returns the maximum neighbor count of a specified agent.
     *
     * @param agentNo The number of the agent whose maximum neighbor count is to
     *                be retrieved.
     * @return The present maximum neighbor count of the agent.
     */
    public int getAgentMaxNeighbors(int agentNo) {
        return agents.get(agentNo).maxNeighbors;
    }

    /**
     * Returns the maximum speed of a specified agent.
     *
     * @param agentNo The number of the agent whose maximum speed is to be
     *                retrieved.
     * @return The present maximum speed of the agent.
     */
    public double getAgentMaxSpeed(int agentNo) {
        return agents.get(agentNo).maxSpeed;
    }

    /**
     * Returns the maximum neighbor distance of a specified agent.
     *
     * @param agentNo The number of the agent whose maximum neighbor distance is
     *                to be retrieved.
     * @return The present maximum neighbor distance of the agent.
     */
    public double getAgentNeighborDistance(int agentNo) {
        return agents.get(agentNo).neighborDistance;
    }

    /**
     * Returns the count of agent neighbors taken into account to compute the
     * current velocity for the specified agent.
     *
     * @param agentNo The number of the agent whose count of agent neighbors is
     *                to be retrieved.
     * @return The count of agent neighbors taken into account to compute the
     * current velocity for the specified agent.
     */
    public int getAgentNumAgentNeighbors(int agentNo) {
        return agents.get(agentNo).agentNeighbors.size();
    }

    /**
     * Returns the count of obstacle neighbors taken into account to compute the
     * current velocity for the specified agent.
     *
     * @param agentNo The number of the agent whose count of obstacle neighbors
     *                is to be retrieved.
     * @return The count of obstacle neighbors taken into account to compute the
     * current velocity for the specified agent.
     */
    public int getAgentNumObstacleNeighbors(int agentNo) {
        return agents.get(agentNo).obstacleNeighbors.size();
    }

    /**
     * Returns the specified obstacle neighbor of the specified agent.
     *
     * @param agentNo    The number of the agent whose obstacle neighbor is to
     *                   be retrieved.
     * @param neighborNo The number of the obstacle neighbor to be retrieved.
     * @return The number of the first vertex of the neighboring obstacle edge.
     */
    public int getAgentObstacleNeighbor(int agentNo, int neighborNo) {
        return agents.get(agentNo).obstacleNeighbors.get(neighborNo).getSecond().id;
    }

    /**
     * Returns the ORCA constraints of the specified agent. The half-plane to
     * the left of each line is the region of permissible velocities with
     * respect to that ORCA constraint.
     *
     * @param agentNo The number of the agent whose ORCA constraints are to be
     *                retrieved.
     * @return A list of lines representing the ORCA constraints.
     */
    public List<Line> getAgentLines(int agentNo) {
        return agents.get(agentNo).lines;
    }

    /**
     * Returns the two-dimensional position of a specified agent.
     *
     * @param agentNo The number of the agent whose two-dimensional position is
     *                to be retrieved.
     * @return The present two-dimensional position of the (center of the)
     * agent.
     */
    public Vector2D getAgentPosition(int agentNo) {
        return agents.get(agentNo).position;
    }

    /**
     * Returns the two-dimensional preferred velocity of a specified agent.
     *
     * @param agentNo The number of the agent whose two-dimensional preferred
     *                velocity is to be retrieved.
     * @return The present two-dimensional preferred velocity of the agent.
     */
    public Vector2D getAgentPreferredVelocity(int agentNo) {
        return agents.get(agentNo).preferredVelocity;
    }

    /**
     * Returns the radius of a specified agent.
     *
     * @param agentNo The number of the agent whose radius is to be retrieved.
     * @return The present radius of the agent.
     */
    public double getAgentRadius(int agentNo) {
        return agents.get(agentNo).radius;
    }

    /**
     * Returns the time horizon of a specified agent.
     *
     * @param agentNo The number of the agent whose time horizon is to be
     *                retrieved.
     * @return The present time horizon of the agent.
     */
    public double getAgentTimeHorizonAgents(int agentNo) {
        return agents.get(agentNo).timeHorizonAgents;
    }

    /**
     * Returns the time horizon with respect to obstacles of a specified agent.
     *
     * @param agentNo The number of the agent whose time horizon with respect to
     *                obstacles is to be retrieved.
     * @return The present time horizon with respect to obstacles of the agent.
     */
    public double getAgentTimeHorizonObstacles(int agentNo) {
        return agents.get(agentNo).timeHorizonObstacles;
    }

    /**
     * Returns the two-dimensional linear velocity of a specified agent.
     *
     * @param agentNo The number of the agent whose two-dimensional linear
     *                velocity is to be retrieved.
     * @return The present two-dimensional linear velocity of the agent.
     */
    public Vector2D getAgentVelocity(int agentNo) {
        return agents.get(agentNo).velocity;
    }

    /**
     * Returns the global time of the simulation.
     *
     * @return The present global time of the simulation (zero initially).
     */
    public double getGlobalTime() {
        return globalTime;
    }

    /**
     * Returns the count of agents in the simulation.
     *
     * @return The count of agents in the simulation.
     */
    public int getNumAgents() {
        return agents.size();
    }

    /**
     * Returns the count of obstacle vertices in the simulation.
     *
     * @return The count of obstacle vertices in the simulation.
     */
    public int getNumObstacleVertices() {
        return obstacles.size();
    }

    /**
     * Returns the two-dimensional position of a specified obstacle vertex.
     *
     * @param vertexNo The number of the obstacle vertex to be retrieved.
     * @return The two-dimensional position of the specified obstacle vertex.
     */
    public Vector2D getObstacleVertex(int vertexNo) {
        return obstacles.get(vertexNo).point;
    }

    /**
     * Returns the number of the obstacle vertex succeeding the specified
     * obstacle vertex in its polygon.
     *
     * @param vertexNo The number of the obstacle vertex whose successor is to
     *                 be retrieved.
     * @return The number of the obstacle vertex succeeding the specified
     * obstacle vertex in its polygon.
     */
    public int getNextObstacleVertexNo(int vertexNo) {
        return obstacles.get(vertexNo).next.id;
    }

    /**
     * Returns the number of the obstacle vertex preceding the specified
     * obstacle vertex in its polygon.
     *
     * @param vertexNo The number of the obstacle vertex whose predecessor is to
     *                 be retrieved.
     * @return The number of the obstacle vertex preceding the specified
     * obstacle vertex in its polygon.
     */
    public int getPreviousObstacleVertexNo(int vertexNo) {
        return obstacles.get(vertexNo).previous.id;
    }

    /**
     * Returns the time step of the simulation.
     *
     * @return The present time step of the simulation.
     */
    public double getTimeStep() {
        return timeStep;
    }

    /**
     * Processes the obstacles that have been added so that they are accounted
     * for in the simulation. Obstacles added to the simulation after this
     * function has been called are not accounted for in the simulation.
     */
    public void processObstacles() {
        kdTree.buildObstacleTree();
    }

    /**
     * Performs a visibility query between the two specified points with respect
     * to the obstacles.
     *
     * @param point1 The first point of the query.
     * @param point2 The second point of the query.
     * @param radius The minimal distance between the line connecting the two
     *               points and the obstacles in order for the points to be
     *               mutually visible (optional). Must be non-negative.
     * @return A boolean specifying whether the two points are mutually visible.
     * Returns true when the obstacles have not been processed.
     */
    public boolean queryVisibility(Vector2D point1, Vector2D point2, double radius) {
        return kdTree.queryVisibility(point1, point2, radius);
    }

    /**
     * Sets the default properties for any new agent that is added.
     *
     * @param neighborDistance     The default maximum distance (center point to
     *                             center point) to other agents a new agent
     *                             takes into account in the navigation. The
     *                             larger this number, the longer he running
     *                             time of the simulation. If the number is too
     *                             low, the simulation will not be safe. Must be
     *                             non-negative.
     * @param maxNeighbors         The default maximum number of other agents a
     *                             new agent takes into account in the
     *                             navigation. The larger this number, the
     *                             longer the running time of the simulation. If
     *                             the number is too low, the simulation will
     *                             not be safe.
     * @param timeHorizonAgents    The default minimal amount of time for which
     *                             a new agent's velocities that are computed by
     *                             the simulation are safe with respect to other
     *                             agents. The larger this number, the sooner an
     *                             agent will respond to the presence of other
     *                             agents, but the less freedom the agent has in
     *                             choosing its velocities. Must be positive.
     * @param timeHorizonObstacles The default minimal amount of time for which
     *                             a new agent's velocities that are computed by
     *                             the simulation are safe with respect to
     *                             obstacles. The larger this number, the sooner
     *                             an agent will respond to the presence of
     *                             obstacles, but the less freedom the agent has
     *                             in choosing its velocities. Must be positive.
     * @param radius               The default radius of a new agent. Must be
     *                             non-negative.
     * @param maxSpeed             The default maximum speed of a new agent.
     *                             Must be non-negative.
     * @param velocity             The default initial two-dimensional linear
     *                             velocity of a new agent.
     */
    public void setAgentDefaults(double neighborDistance, int maxNeighbors, double timeHorizonAgents, double timeHorizonObstacles, double radius, double maxSpeed, Vector2D velocity) {
        if (defaultAgent == null) {
            defaultAgent = new Agent();
        }

        defaultAgent.maxNeighbors = maxNeighbors;
        defaultAgent.maxSpeed = maxSpeed;
        defaultAgent.neighborDistance = neighborDistance;
        defaultAgent.radius = radius;
        defaultAgent.timeHorizonAgents = timeHorizonAgents;
        defaultAgent.timeHorizonObstacles = timeHorizonObstacles;
        defaultAgent.velocity = velocity;
    }

    /**
     * Sets the maximum neighbor count of a specified agent.
     *
     * @param agentNo      The number of the agent whose maximum neighbor count
     *                     is to be modified.
     * @param maxNeighbors The replacement maximum neighbor count.
     */
    public void setAgentMaxNeighbors(int agentNo, int maxNeighbors) {
        agents.get(agentNo).maxNeighbors = maxNeighbors;
    }

    /**
     * Sets the maximum speed of a specified agent.
     *
     * @param agentNo  The number of the agent whose maximum speed is to be
     *                 modified.
     * @param maxSpeed The replacement maximum speed. Must be non-negative.
     */
    public void setAgentMaxSpeed(int agentNo, double maxSpeed) {
        agents.get(agentNo).maxSpeed = maxSpeed;
    }

    /**
     * Sets the maximum neighbor distance of a specified agent.
     *
     * @param agentNo          The number of the agent whose maximum neighbor
     *                         distance is to be modified.
     * @param neighborDistance The replacement maximum neighbor distance. Must
     *                         be non-negative.
     */
    public void setAgentNeighborDistance(int agentNo, double neighborDistance) {
        agents.get(agentNo).neighborDistance = neighborDistance;
    }

    /**
     * Sets the two-dimensional position of a specified agent.
     *
     * @param agentNo  The number of the agent whose two-dimensional position is
     *                 to be modified.
     * @param position The replacement of the two-dimensional position.
     */
    public void setAgentPosition(int agentNo, Vector2D position) {
        agents.get(agentNo).position = position;
    }

    /**
     * Sets the two-dimensional preferred velocity of a specified agent.
     *
     * @param agentNo           The number of the agent whose two-dimensional
     *                          preferred velocity is to be modified.
     * @param preferredVelocity The replacement of the two-dimensional preferred
     *                          velocity.
     */
    public void setAgentPreferredVelocity(int agentNo, Vector2D preferredVelocity) {
        agents.get(agentNo).preferredVelocity = preferredVelocity;
    }

    /**
     * Sets the radius of a specified agent.
     *
     * @param agentNo The number of the agent whose radius is to be modified.
     * @param radius  The replacement radius. Must be non-negative.
     */
    public void setAgentRadius(int agentNo, double radius) {
        agents.get(agentNo).radius = radius;
    }

    /**
     * Sets the time horizon of a specified agent with respect to other agents.
     *
     * @param agentNo           The number of the agent whose time horizon is to
     *                          be modified.
     * @param timeHorizonAgents The replacement time horizon with respect to
     *                          other agents. Must be positive.
     */
    public void setAgentTimeHorizonAgents(int agentNo, double timeHorizonAgents) {
        agents.get(agentNo).timeHorizonAgents = timeHorizonAgents;
    }

    /**
     * Sets the time horizon of a specified agent with respect to obstacles.
     *
     * @param agentNo              The number of the agent whose time horizon
     *                             with respect to obstacles is to be modified.
     * @param timeHorizonObstacles The replacement time horizon with respect to
     *                             obstacles. Must be positive.
     */
    public void setAgentTimeHorizonObstacles(int agentNo, double timeHorizonObstacles) {
        agents.get(agentNo).timeHorizonObstacles = timeHorizonObstacles;
    }

    /**
     * Sets the two-dimensional linear velocity of a specified agent.
     *
     * @param agentNo  The number of the agent whose two-dimensional linear
     *                 velocity is to be modified.
     * @param velocity The replacement two-dimensional linear velocity.
     */
    public void setAgentVelocity(int agentNo, Vector2D velocity) {
        agents.get(agentNo).velocity = velocity;
    }

    /**
     * Sets the global time of the simulation.
     *
     * @param globalTime The global time of the simulation.
     */
    public void setGlobalTime(double globalTime) {
        this.globalTime = globalTime;
    }

    /**
     * Sets the time step of the simulation.
     *
     * @param timeStep The time step of the simulation. Must be positive.
     */
    public void setTimeStep(double timeStep) {
        this.timeStep = timeStep;
    }

    /**
     * Constructs and initializes a simulation.
     */
    private Simulator() {
    }
}
