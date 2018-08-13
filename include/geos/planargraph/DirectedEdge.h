/**********************************************************************
 *
 * GEOS - Geometry Engine Open Source
 * http://geos.osgeo.org
 *
 * Copyright (C) 2001-2002 Vivid Solutions Inc.
 * Copyright (C) 2005-2006 Refractions Research Inc.
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the GNU Lesser General Public Licence as published
 * by the Free Software Foundation.
 * See the COPYING file for more information.
 *
 **********************************************************************/

#ifndef GEOS_PLANARGRAPH_DIRECTEDEDGE_H
#define GEOS_PLANARGRAPH_DIRECTEDEDGE_H

#include <geos/export.h>
#include <geos/planargraph/GraphComponent.h> // for inheritance
#include <geos/geom/Coordinate.h> // for composition
#include <geos/planargraph/NodeMap.h>


#include <vector> // for typedefs
#include <list> // for typedefs
#include <memory> // for typedefs

// Forward declarations
namespace geos {
	namespace planargraph {
		class Edge;
		class Node;
	}
}

using geos::planargraph::NodeMap;

namespace geos {
namespace planargraph { // geos.planargraph

/**
 * \brief Represents a directed edge in a PlanarGraph.
 *
 * A DirectedEdge may or may not have a reference to a parent Edge
 * (some applications of planar graphs may not require explicit Edge
 * objects to be created). Usually a client using a PlanarGraph
 * will subclass DirectedEdge to add its own application-specific
 * data and methods.
 */
class GEOS_DLL DirectedEdge: public GraphComponent {

public:

  friend std::ostream& operator << (std::ostream&, const DirectedEdge&);

	typedef std::shared_ptr<DirectedEdge> DirectedEdgePtr;
	typedef std::list<DirectedEdgePtr> NonConstList;
	typedef std::list<const DirectedEdgePtr> ConstList;
	typedef std::vector<DirectedEdgePtr> DirectedEdges;
	typedef std::vector<const DirectedEdgePtr> ConstVect;

	typedef NodeMap::NodePtr NodePtr;


protected:
	Edge* m_parentEdge;
	NodePtr m_from;
	NodePtr m_to;
	geom::Coordinate m_p0, m_p1;
	DirectedEdgePtr m_sym;  // optional
	bool m_edgeDirection;
	int m_quadrant;
	double m_angle;

public:

	/**
	 * \brief
	 * Returns a List containing the parent Edge (possibly null)
	 * for each of the given DirectedEdges.
	 *
	 * NOTE: ownership of the returned vector is left to
	 * the caller, see the equivalent function taking a vector
	 * reference to avoid this.
	 */
	static std::vector<Edge*> toEdges(
		DirectedEdges& dirEdges);

	/**
	 * \brief Constructs a DirectedEdge connecting the <code>from</code>
	 * node to the <code>to</code> node.
	 *
	 * @param directionPt specifies this DirectedEdge's direction
	 *                    (given by an imaginary line from the
	 *		      <code>from</code> node to
	 *		      <code>directionPt</code>)
	 * @param edgeDirection whether this DirectedEdge's direction
	 *		       is the same as or opposite to that of the
	 *		       parent Edge (if any)
	 */
	DirectedEdge(
			NodePtr newFrom,
		 	NodePtr newTo,
			const geom::Coordinate &directionPt,
			bool newEdgeDirection);

	/**
	 * \brief Returns this DirectedEdge's parent Edge,
	 * or null if it has none.
	 */
	Edge* parentEdge() const;
	/**
	 * \brief Associates this DirectedEdge with an Edge
	 * (possibly null, indicating no associated Edge).
	 */
	void set_parentEdge(Edge* newParentEdge);

	/**
	 * \brief Returns 0, 1, 2, or 3, indicating the quadrant in which
	 * this DirectedEdge's orientation lies.
	 */
	int getQuadrant() const;

	/**
	 * \brief Returns a point to which an imaginary line is drawn
	 * from the from-node to specify this DirectedEdge's orientation.
	 */
	const geom::Coordinate& getDirectionPt() const;

	/**
	 * \brief Returns whether the direction of the parent Edge (if any)
	 * is the same as that of this Directed Edge.
	 */
	bool getEdgeDirection() const;

	/**
	 * \brief Returns the node from which this DirectedEdge leaves.
	 */
	NodePtr getFromNode() const;

	/**
	 * \brief Returns the node to which this DirectedEdge goes.
	 */
	NodePtr getToNode() const;

	/**
	 * \brief
	 * Returns the coordinate of the from-node.
	 */
	geom::Coordinate& getCoordinate() const;

	/**
	 * \brief
	 * Returns the angle that the start of this DirectedEdge makes
	 * with the positive x-axis, in radians.
	 */
	double getAngle() const;

	/**
	 * \brief
	 * Returns the symmetric DirectedEdge -- the other DirectedEdge
	 * associated with this DirectedEdge's parent Edge.
	 */
	DirectedEdgePtr getSym() const;

	/**
	 * \brief
	 * Sets this DirectedEdge's symmetric DirectedEdge, which runs
	 * in the opposite direction.
	 */
	void setSym(DirectedEdgePtr newSym);
	/**
	 * \brief
	 * Unsets this DirectedEdge's symmetric DirectedEdge
	 */
	void resetSym();

	/**
	 * \brief
	 * Returns 1 if this DirectedEdge has a greater angle with the
	 * positive x-axis than b", 0 if the DirectedEdges are collinear,
	 * and -1 otherwise.
	 *
	 * Using the obvious algorithm of simply computing the angle is
	 * not robust, since the angle calculation is susceptible to roundoff.
	 * A robust algorithm is:
	 *
	 * - first compare the quadrants.
	 *   If the quadrants are different, it it
	 *   trivial to determine which std::vector is "greater".
	 * - if the vectors lie in the same quadrant, the robust
	 *   RobustCGAlgorithms::computeOrientation(Coordinate, Coordinate, Coordinate)
	 *   function can be used to decide the relative orientation of
	 *   the vectors.
	 *
	 */
	int compareTo(const DirectedEdge obj) const;

	/**
	 * \brief
	 * Returns 1 if this DirectedEdge has a greater angle with the
	 * positive x-axis than b", 0 if the DirectedEdges are collinear,
	 * and -1 otherwise.
	 *
	 * Using the obvious algorithm of simply computing the angle is
	 * not robust, since the angle calculation is susceptible to roundoff.
	 * A robust algorithm is:
	 *
	 * - first compare the quadrants.
	 *   If the quadrants are different, it it trivial to determine
	 *   which std::vector is "greater".
	 * - if the vectors lie in the same quadrant, the robust
	 *   RobustCGAlgorithms::computeOrientation(Coordinate, Coordinate, Coordinate)
	 *   function can be used to decide the relative orientation of
	 *   the vectors.
	 *
	 */
	int compareDirection(const DirectedEdge e) const;

	/**
	 * \brief
	 * Prints a detailed string representation of this DirectedEdge
	 * to the given PrintStream.
	 */
	std::string print() const;

  virtual long getLabel() const {return -1;}
  virtual void setLabel(long) { }
  virtual DirectedEdgePtr getNext() const {return nullptr;}
  virtual void setNext(DirectedEdgePtr&) { }

#ifdef GEOS_USEDEPRECATED
	/** @name deprecated */
	///@{
	/**
	 * \brief
	 * Add parent Edge (possibly null) of each of the given DirectedEdges
	 * to the given parentEdges vector.
	 *
	 * NOTE: parents are pushed to the parentEdges vector, make sure
	 * it is empty if index-based corrispondence is important.
	 */
	// [[deprecated]]
	static void toEdges( DirectedEdges& dirEdges,
			std::vector<Edge*>& parentEdges);

	// [[deprecated]]
	Edge* getEdge() const;

	// [[deprecated]]
	void setEdge(Edge* newParentEdge);
	///@}
#endif


};




} // namespace geos::planargraph
} // namespace geos

#endif // GEOS_PLANARGRAPH_DIRECTEDEDGE_H
