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

#ifndef GEOS_PLANARGRAPH_NODE_H
#define GEOS_PLANARGRAPH_NODE_H

#include <geos/export.h>

#include <geos/planargraph/GraphComponent.h> // for inheritance
#include <geos/planargraph/DirectedEdgeStar.h>
#include <geos/planargraph/DirectedEdge.h>
#include <geos/planargraph/detail.hpp>
#include <algorithm>

// Forward declarations
namespace geos {
  namespace geom {
    class Coordinate;
  }
}

namespace geos {
namespace planargraph { // geos.planargraph

/**
 * \brief A node in a PlanarGraph is a location where 0 or more Edge meet.
 *
 * A node is connected to each of its incident Edges via an outgoing
 * DirectedEdge. Some clients using a <code>PlanarGraph</code> may want to
 * subclass <code>Node</code> to add their own application-specific
 * data and methods.
 *
 */
class GEOS_DLL Node: public GraphComponent {
private:

	/// The location of this Node
	geom::Coordinate pt;

	/// The collection of DirectedEdges that leave this Node
	DirectedEdgeStar deStar;

public:
  typedef std::shared_ptr<DirectedEdge> DirectedEdgePtr;
  struct DirectedEdgeCmp {
    bool operator() (const DirectedEdgePtr lhs, const DirectedEdgePtr rhs) const
    {
      return lhs->compareTo(*rhs) < 0;
    }
  } DirectedEdgePtr_less;


	friend std::ostream& operator << (std::ostream& os, const Node&);


	/// Constructs a Node with the given location.
	Node(const geom::Coordinate& newPt)
		:
		pt(newPt)
		{ }

	~Node() override {
	}

	/**
	 * \brief
	 * Constructs a Node with the given location and
	 * collection of outgoing DirectedEdges.
	 * Takes ownership of the given DirectedEdgeStar!!
	 */
	Node(geom::Coordinate& newPt, DirectedEdgeStar newDeStar)
		:
		pt(newPt),
		deStar(newDeStar)
		{}

	/**
	 * \brief Returns the location of this Node.
	 */
	geom::Coordinate& getCoordinate() {
		return pt;
	}

	/**
	 * \brief Adds an outgoing DirectedEdge to this Node.
	 */
	void addOutEdge(DirectedEdgeStar::DirectedEdgePtr de) {
		deStar.add(de);
	}

	/**
	 * \brief Returns the collection of DirectedEdges that
	 * leave this Node.
	 */
	DirectedEdgeStar getSortedOutEdges() {
    std::sort(deStar.begin(), deStar.end(), DirectedEdgePtr_less);
    return deStar;
  }

	DirectedEdgeStar getOutEdges() { return deStar; }
	const DirectedEdgeStar& getOutEdges() const { return deStar; }

	/**
	 * \brief Returns the number of edges around this Node.
	 */
	size_t getDegree() const {
		return deStar.getDegree();
	}

#if 0
  template <typename T>
	size_t getDegreeNonDeleted() const;
#endif

  template <typename T>
	size_t getDegree(long label) const;

  size_t getDegreeNonDeleted() const;


  /** @brief mark all edges as deleted*/
	void markAll(bool value);

	/**
	 * \brief Returns the number of edges around this Node.
	 */
	bool hasDegree(size_t degree) const;


#ifdef GEOS_USEDEPRECATED
	/** @name deprecated */
	///@{
	/** \brief
	 * Returns all Edges that connect the two nodes (which are
	 * assumed to be different).
	 *
	 * Note: returned vector is newly allocated, ownership to
	 * the caller.
	 *
	 * NOT used and not tested
	 */
	// [[deprecated]]
	static std::vector<Edge*>* getEdgesBetween(Node *node0,
			Node *node1);

	/**
	 * \brief Returns the zero-based index of the given Edge,
	 * after sorting in ascending order by angle with
	 * the positive x-axis.
	 */
	// [[deprecated]]
	int getIndex(Edge *edge) {
		return deStar.getIndex(edge);
	}
	///@}
#endif
};


template <typename T>
size_t
Node::getDegree(long label) const {
  size_t degree = 0;
  for (const auto e : deStar) {
    if (safe_cast<T*>(e)->getLabel() == label) ++degree;
  }
  return degree;
}


/// Print a Node
std::ostream& operator<<(std::ostream& os, const Node& n);


/// For backward compatibility
//typedef Node planarNode;

} // namespace geos::planargraph
} // namespace geos

#endif // GEOS_PLANARGRAPH_NODE_H
