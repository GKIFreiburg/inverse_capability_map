#include "inverse_capability_map/InverseCapabilityOcTree.h"
#include "inverse_capability_map/InverseCapabilityOcTreeNode.h"
#include <ros/ros.h>

bool InverseCapabilityOcTree::writeFile(const std::string &filename)
{
    std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);

    if (!file.is_open())
    {
        OCTOMAP_ERROR_STR("Filestream to " << filename << " not open, nothing written.");
        return false;
    }
    else
    {
        if (!write(file))
        {
            file.close();
            OCTOMAP_ERROR_STR("Could not write to " << filename);
            return false;
        }
        file << std::endl << "group_name " << group_name_ << std::endl;
        file << "base_name " << base_name_ << std::endl;
        file << "tip_name " << tip_name_ << std::endl;
        file << "theta_resolution " << theta_resolution_ << std::endl;
        file.setf( std::ios::fixed, std::ios::floatfield );
        file << "maximum_percent " << maximum_percent_ << std::endl;
        file.close();
    }
    return true;
}

InverseCapabilityOcTree* InverseCapabilityOcTree::readFile(const std::string &filename)
{
    std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);

    if (!file.is_open())
    {
        OCTOMAP_ERROR_STR("Filestream to " << filename << " not open, nothing read.");
        return NULL;
    }

    AbstractOcTree* abstractTree = AbstractOcTree::read(file);

    if (abstractTree == NULL || abstractTree->getTreeType() != "InverseCapabilityOcTree")
    {
        OCTOMAP_ERROR_STR("Could not read " << filename << ". Is the file a valid inverse capability map?");
        return NULL;
    }
    std::string qualifier;
    std::string groupName;
    std::string baseName;
    std::string tipName;
    std::string theta_resolution;
    std::string minimum_percent;
    std::string maximum_percent;

    while(!file.eof())
    {
        file >> qualifier;
        if (qualifier == "group_name")
        {
            file.ignore(1, ' ');
            std::getline(file, groupName);
        }
        else if (qualifier == "base_name")
        {
            file.ignore(1, ' ');
            std::getline(file, baseName);
        }
        else if (qualifier == "tip_name")
        {
            file.ignore(1, ' ');
            std::getline(file, tipName);
        }
        else if (qualifier == "theta_resolution")
        {
            file.ignore(1, ' ');
            std::getline(file, theta_resolution);
        }
        else if (qualifier == "maximum_percent")
        {
            file.ignore(1, ' ');
            std::getline(file, maximum_percent);
        }
    }

    file.close();

    InverseCapabilityOcTree* tree = dynamic_cast<InverseCapabilityOcTree*>(abstractTree);

    unsigned int theta_res = atoi(theta_resolution.c_str());
    double max_percent = atof(maximum_percent.c_str());

    tree->setGroupName(groupName);
    tree->setBaseName(baseName);
    tree->setTipName(tipName);
    tree->setThetaResolution(theta_res);
    tree->setMaximumPercent(max_percent);

    return tree;
}

InverseCapabilityOcTreeNode* InverseCapabilityOcTree::setNodeInverseCapability(const OcTreeKey &key, const InverseCapability &inv_capa)
{
    bool createdRoot = false;
    if (this->root == NULL){
      this->root = new InverseCapabilityOcTreeNode();
      this->tree_size++;
      createdRoot = true;
    }

    return setNodeInverseCapabilityRecurs(this->root, createdRoot, key, 0, inv_capa);
}

InverseCapabilityOcTreeNode* InverseCapabilityOcTree::setNodeInverseCapability(const double &x, const double &y, const double &z,
										const std::map<double, double> &thetas)
{
    OcTreeKey key;
    // NOTE: there is a bug in creating a key. Floating point precision seems to cause the error
    // adding a small amount (1% of resolution) to x, y and z should handle this
    double correctionValue = resolution/100.0;
    if (!this->coordToKeyChecked(x + correctionValue, y + correctionValue, z + correctionValue, key))
    {
        return NULL;
    }
    return setNodeInverseCapability(key, InverseCapability(thetas));
}

InverseCapabilityOcTreeNode* InverseCapabilityOcTree::setNodeInverseCapability(const double &x, const double &y,
                                        const double &z, const InverseCapability &inv_capa)
{
    OcTreeKey key;
    // NOTE: there is a bug in creating a key. Floating point precision seems to cause the error
    // adding a small amount (1% of resolution) to x, y and z should handle this
    double correctionValue = resolution/100.0;
    if (!this->coordToKeyChecked(x + correctionValue, y + correctionValue, z + correctionValue, key))
    {
        return NULL;
    }
    return setNodeInverseCapability(key, inv_capa);
}

InverseCapability InverseCapabilityOcTree::getNodeInverseCapability(const double &x, const double &y, const double &z) const
{
    // NOTE: there is a bug in creating a key. Floating point precision seems to cause the error
    // adding a small amount (1% of resolution) to x, y and z should handle this
    double correctionValue = resolution/100.0;
    InverseCapabilityOcTreeNode* n = search(x + correctionValue, y + correctionValue, z + correctionValue);
    if(!n)
        return InverseCapability();
    return n->getInverseCapability();
}

const std::map<double, double>* InverseCapabilityOcTree::getThetasPercent(const double &x, const double &y, const double &z) const
{
    double correctionValue = resolution/100.0;
    InverseCapabilityOcTreeNode* n = search(x + correctionValue, y + correctionValue, z + correctionValue);
	if(!n)
		return NULL;
	return &n->getInverseCapability().getThetasPercent();
}

double InverseCapabilityOcTree::getThetaPercent(const double &x, const double &y, const double &z, const double &theta) const
{
    double correctionValue = resolution/100.0;
    InverseCapabilityOcTreeNode* n = search(x + correctionValue, y + correctionValue, z + correctionValue);
	if(!n)
		return 0.0;
	return n->getInverseCapability().getThetaPercent(theta);
}

const std::pair<double, double>* InverseCapabilityOcTree::getMaxThetaPercent(const double &x, const double &y, const double &z) const
{
    double correctionValue = resolution/100.0;
    InverseCapabilityOcTreeNode* n = search(x + correctionValue, y + correctionValue, z + correctionValue);
	if(!n)
		return NULL;
	return &n->getInverseCapability().getMaxThetaPercent();
}

std::map<double, double> InverseCapabilityOcTree::getThetasWithMinPercent(const double &x, const double &y, const double &z, const double &percent) const
{
    double correctionValue = resolution/100.0;
    InverseCapabilityOcTreeNode* n = search(x + correctionValue, y + correctionValue, z + correctionValue);
	if(!n)
		return std::map<double, double>();
	return n->getInverseCapability().getThetasWithMinPercent(percent);
}

InverseCapabilityOcTreeNode* InverseCapabilityOcTree::setNodeInverseCapabilityRecurs(InverseCapabilityOcTreeNode* node,
		bool node_just_created, const OcTreeKey& key, unsigned int depth, const InverseCapability &inv_capa)
{
    unsigned int pos = computeChildIdx(key, this->tree_depth - 1 - depth);
    bool created_node = false;

    ROS_ASSERT(node);

    // follow down to last level
    if (depth < this->tree_depth)
    {
        if (!node->childExists(pos))
        {
            // child does not exist, but maybe it's a pruned node?
            if ((!node->hasChildren()) && !node_just_created)
            {
                // current node does not have children AND it is not a new node
                // -> expand pruned node
                node->expandNode();
                this->tree_size += 8;
                this->size_changed = true;
            }
            else
            {
                // not a pruned node, create requested child
                node->createChild(pos);
                this->tree_size++;
                this->size_changed = true;
                created_node = true;
            }
        }

        InverseCapabilityOcTreeNode* retval = setNodeInverseCapabilityRecurs(node->getChild(pos), created_node, key, depth+1, inv_capa);
        // prune node if possible, otherwise set own probability
        // note: combining both did not lead to a speedup!
        if (node->pruneNode())
        {
            this->tree_size -= 8;
            // return pointer to current parent (pruned), the just updated node no longer exists
            retval = node;
        }
        else
        {
            // TODO: maybe set some kind of maxCapability
            node->setInverseCapability(inv_capa);
        }

        return retval;
    }

    // at last level, update node, end of recursion
    else
    {
        node->setInverseCapability(inv_capa);
        return node;
    }
}

InverseCapabilityOcTree::StaticMemberInitializer InverseCapabilityOcTree::inverseCapabilityOcTreeMemberInit;
