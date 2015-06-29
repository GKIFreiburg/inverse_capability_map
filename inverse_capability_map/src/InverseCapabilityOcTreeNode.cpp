#include "inverse_capability_map/InverseCapabilityOcTreeNode.h"
#include <ros/ros.h>
#include <algorithm>
//#include <cmath>

/**************************************************************************************************
 *
 * 										InverseCapability
 *
 *************************************************************************************************/

InverseCapability::InverseCapability()
{
}

InverseCapability::InverseCapability(const std::map<double, double> &thetas)
{
	thetas_ = thetas;
}

double InverseCapability::getThetaPercent(const double theta)
{
	std::map<double, double>::iterator it;
	it = thetas_.find(theta);
	if (it != thetas_.end())
		return it->second;
	else
		return 0.0;
}

bool InverseCapability::operator==(const InverseCapability &other) const
{
    return thetas_.size() == other.thetas_.size()
        && std::equal(thetas_.begin(), thetas_.end(),
                      other.thetas_.begin());
}

bool InverseCapability::operator!=(const InverseCapability &other) const
{
    return !(this->thetas_ == other.thetas_);
}

const std::pair<double, double> & InverseCapability::getMaxThetaPercent()
{
	std::map<double, double>::iterator it;
	it = std::max_element(thetas_.begin(), thetas_.end(), LessThanSecond());
	return *it;
}

std::map<double, double> InverseCapability::getThetasWithMinPercent(double minPercent) const
{
	std::map<double, double> ret;
	std::map<double, double>::const_iterator it;
	for (it = thetas_.begin(); it != thetas_.end(); it++)
	{
		if (it->second > minPercent)
			ret.insert(*it);
	}
	return ret;
}

/**************************************************************************************************
 *
 * 									InverseCapabilityOcTreeNode
 *
 *************************************************************************************************/

InverseCapabilityOcTreeNode::InverseCapabilityOcTreeNode()
{
}

InverseCapabilityOcTreeNode::InverseCapabilityOcTreeNode(InverseCapability inv_capa)
	: OcTreeDataNode<InverseCapability>(inv_capa)
{
}

InverseCapabilityOcTreeNode::InverseCapabilityOcTreeNode(const InverseCapabilityOcTreeNode &rhs)
	: OcTreeDataNode<InverseCapability>(rhs)
{
}

InverseCapabilityOcTreeNode::~InverseCapabilityOcTreeNode()
{
}

bool InverseCapabilityOcTreeNode::createChild(unsigned int i)
{
    if (children == NULL)
    {
        allocChildren();
    }
    assert (children[i] == NULL);
    children[i] = new InverseCapabilityOcTreeNode();
    return true;
}

std::ostream& InverseCapabilityOcTreeNode::writeValue(std::ostream &s) const
{
    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (childExists(i))
        {
          children[i] = 1;
        }
        else
        {
          children[i] = 0;
        }
    }
    char children_char = (char)children.to_ulong();

    // buffer inverse capability data
    std::map<double, double> thetas = value.getThetasPercent();
    unsigned int size = thetas.size();

    // write node data
    s.write((const char*)&size, sizeof(unsigned int));
    std::map<double, double>::iterator it;
    double theta, percent;
    for (it = thetas.begin(); it != thetas.end(); it++)
    {
    	theta = it->first;
    	percent = it->second;
    	s.write((const char*)&theta  , sizeof(double));
    	s.write((const char*)&percent, sizeof(double));
    }
    s.write((char*)&children_char, sizeof(char)); // child existence

    // write existing children
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (children[i] == 1)
        {
            this->getChild(i)->writeValue(s);
        }
    }
    return s;
}

std::istream& InverseCapabilityOcTreeNode::readValue(std::istream &s)
{
    // buffer for capabilities' data
    unsigned int size;
    std::map<double, double> thetas;
    char children_char;

    // read node data
    s.read((char*)&size, sizeof(unsigned int));

    double theta, percent;
    for (unsigned int i = 0; i < size; i++)
    {
    	s.read((char*)&theta  , sizeof(double));
    	s.read((char*)&percent, sizeof(double));
    	thetas.insert(std::make_pair(theta, percent));
    }
    s.read((char*)&children_char, sizeof(char)); // child existence

    // insert buffered data into node
    value.setThetasPercent(thetas);

    // read existing children
    std::bitset<8> children ((unsigned long long)children_char);
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (children[i] == 1)
        {
            createChild(i);
            getChild(i)->readValue(s);
        }
    }
    return s;
}


