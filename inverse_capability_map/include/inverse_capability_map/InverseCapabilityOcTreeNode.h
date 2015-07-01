#ifndef INVERSECAPABILITYOCTREENODE_H
#define INVERSECAPABILITYOCTREENODE_H

#include <gtest/gtest.h>
#include <octomap/OcTreeDataNode.h>
#include <map>

using namespace octomap;

class InverseCapability
{
  public:
    /// Class to find max value in map
    struct LessThanSecond
    {
        template <typename Lhs, typename Rhs>
        bool operator()(const Lhs& lhs, const Rhs& rhs) const
        {
            return lhs.second < rhs.second;
        }
    };

  public:

    FRIEND_TEST(InverseCapability, constructor);
	InverseCapability();
	InverseCapability(const std::map<double, double> &thetas);

    // setter and getter
    void setThetasPercent(const std::map<double, double> &thetas) { thetas_ =  thetas; }
    const std::map<double, double> & getThetasPercent() const { return thetas_; }

    void setThetaPercent(const std::pair<double, double> &p) { thetas_.insert(p); }
    double getThetaPercent(const double theta);

//    FRIEND_TEST(Capability, equalityOperators);
    bool operator==(const InverseCapability &other) const;
    bool operator!=(const InverseCapability &other) const;
    // add to inverseCapabilities
    InverseCapability operator+(const InverseCapability &other) const;

    void normalize(const double& value);

    // Return the <theta, percent> with the highest percentage
//    FRIEND_TEST(Capability, isDirectionPossible);
    const std::pair<double, double> & getMaxThetaPercent();

    // Return all thetas (<theta, percent>) with minimum percent above minPercent
    std::map<double, double> getThetasWithMinPercent(double minPercent) const;

  protected:

    // if object is reachable from given theta, then store [theta, percent] into map
    std::map<double, double> thetas_;
};


class InverseCapabilityOcTreeNode : public OcTreeDataNode<InverseCapability>
{
  public:

    // Constructors
	InverseCapabilityOcTreeNode();
	InverseCapabilityOcTreeNode(InverseCapability inv_capa);
	InverseCapabilityOcTreeNode(const InverseCapabilityOcTreeNode &rhs);

    ~InverseCapabilityOcTreeNode();

//    FRIEND_TEST(CapabilityOcTreeNode, equalityOperator);
    bool operator==(const InverseCapabilityOcTreeNode &rhs) const
    {
        return (rhs.value == value);
    }

    // children

//    FRIEND_TEST(CapabilityOcTreeNode, children);
    bool createChild(unsigned int i);

    inline InverseCapabilityOcTreeNode* getChild(unsigned int i)
    {
        return static_cast<InverseCapabilityOcTreeNode*>(OcTreeDataNode<InverseCapability>::getChild(i));
    }

    inline const InverseCapabilityOcTreeNode* getChild(unsigned int i) const
    {
        return static_cast<const InverseCapabilityOcTreeNode*>(OcTreeDataNode<InverseCapability>::getChild(i));
    }

//    // TODO: should not be overwritten, node only gets pruned when value is equal for all children (uncomment if problems arise)
//    // bool collapsible() { return false; }
//    // bool pruneNode() { return false; }
//    // void expandNode() { }
//
//    FRIEND_TEST(CapabilityOcTreeNode, set_getCapability);
//    // setter/getter for Capability (value derived from OcTreeDataNode)
    inline void setInverseCapability(InverseCapability inv_capa) { value = inv_capa; }
    inline void setInverseCapability(const std::map<double, double> &thetas)
    {
        value = InverseCapability(thetas);
    }

    inline InverseCapability getInverseCapability() const { return value; }

    inline void normalize(const double& val) { value.normalize(val); }
//
//    // TODO: is isCapabilitySet() needed? If yes, uncomment
//    // has a capability been set?
//    /*
//    inline bool isCapabilitySet() const
//    {
//        return (value.getType() != EMPTY);
//    }
//    */
//
//    // file I/O
    std::ostream& writeValue(std::ostream &s) const;
    std::istream& readValue(std::istream &s);

};

#endif // INVERSECAPABILITYOCTREENODE_H
