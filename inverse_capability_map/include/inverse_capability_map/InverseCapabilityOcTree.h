#ifndef INVERSECAPABILITYOCTREE_H
#define INVERSECAPABILITYOCTREE_H

#include "inverse_capability_map/InverseCapabilityOcTreeNode.h"
#include <gtest/gtest.h>
#include <octomap/OcTreeBase.h>
#include <fstream>
#include <string>
#include <iostream>

using namespace octomap;

// tree definition
class InverseCapabilityOcTree : public OcTreeBase<InverseCapabilityOcTreeNode>
{
  public:

    // Default constructor, sets resolution of leafs
    FRIEND_TEST(InverseCapabilityOcTree, constructor);
    InverseCapabilityOcTree(double resolution) : OcTreeBase<InverseCapabilityOcTreeNode>(resolution), maximum_percent_(-1.0) {}

    // virtual constructor: creates a new object of same type
    // (Covariant return type requires an up-to-date compiler)
    InverseCapabilityOcTree* create() const { return new InverseCapabilityOcTree(resolution); }

    std::string getTreeType() const { return "InverseCapabilityOcTree"; }

    // writes the InverseCapabilityOcTree to file
    bool writeFile(const std::string &filename);

    // creates a new CapabilityOcTree from given file (you need to delete the created tree yourself)
    static InverseCapabilityOcTree* readFile(const std::string &filename);

    // returns the position at which the given coordinate ends up in the tree
    inline double getAlignment(double coordinate) { return keyToCoord(coordToKey(coordinate)); }

//    FRIEND_TEST(CapabilityOcTree, set_getNodeCapability);
    // set node inverse capability at given key or coordinate. Replaces previous inverse capability.
    InverseCapabilityOcTreeNode* setNodeInverseCapability(const OcTreeKey &key, const InverseCapability &inv_capa);

    InverseCapabilityOcTreeNode* setNodeInverseCapability(const double &x, const double &y, const double &z,
    										const std::map<double, double> &thetas);

    InverseCapabilityOcTreeNode* setNodeInverseCapability(const double &x, const double &y,
                                            const double &z, const InverseCapability &inv_capa);

    // get node inverse capability at given coordinate
    InverseCapability getNodeInverseCapability(const double &x, const double &y, const double &z) const;

    // convenience functions
//    FRIEND_TEST(CapabilityOcTree, isPosePossible);
    const std::map<double, double>* getThetasPercent(const double &x, const double &y, const double &z) const;
    double getThetaPercent(const double &x, const double &y, const double &z, const double &theta) const;
    const std::pair<double, double>* getMaxThetaPercent(const double &x, const double &y, const double &z) const;
    std::map<double, double> getThetasWithMinPercent(const double &x, const double &y, const double &z, const double &percent) const;


    //std::vector<octomath::Vector3> CapabilityOcTree::getPositionsWithMinReachablePercent(double percent)



    void setGroupName(const std::string &name) { group_name_ = name; }
    std::string getGroupName() const { return group_name_; }

    void setBaseName(const std::string &name) { base_name_ = name; }
    std::string getBaseName() const { return base_name_; }

    void setTipName(const std::string &name) { tip_name_ = name; }
    std::string getTipName() const { return tip_name_; }

    void setThetaResolution(const unsigned int &theta_resolution) { theta_resolution_ = theta_resolution; }
    unsigned int getThetaResolution() const { return theta_resolution_; }

    void setMaximumPercent(const double& percent) { maximum_percent_ = percent; }
    double getMaximumPercent() const { return maximum_percent_; }

  protected:

    InverseCapabilityOcTreeNode* setNodeInverseCapabilityRecurs(InverseCapabilityOcTreeNode* node, bool node_just_created, const OcTreeKey& key,
                           unsigned int depth, const InverseCapability &inv_capa);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer
    {
      public:
        StaticMemberInitializer()
        {
            InverseCapabilityOcTree* tree = new InverseCapabilityOcTree(0.1);
            AbstractOcTree::registerTreeType(tree);
         }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer inverseCapabilityOcTreeMemberInit;

  private:

    std::string group_name_;
    std::string base_name_;
    std::string tip_name_;
    unsigned int theta_resolution_;
    double maximum_percent_;
};


#endif // INVERSECAPABILITYOCTREE_H
