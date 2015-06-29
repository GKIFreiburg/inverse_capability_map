#include <iostream>
#include <gtest/gtest.h>
#include "inverse_capability_map/InverseCapabilityOcTreeNode.h"
#include "inverse_capability_map/InverseCapabilityOcTree.h"


TEST(InverseCapability, constructor)
{
	ASSERT_TRUE(false);
//    // check if constructors do their job
//    Capability cap1;
//
//    ASSERT_EQ(EMPTY, cap1._type);
//    ASSERT_EQ(0.0, cap1._phi);
//    ASSERT_EQ(0.0, cap1._theta);
//    ASSERT_EQ(0.0, cap1._halfOpeningAngle);
//    ASSERT_EQ(0.0, cap1._shapeFitError);
//
//    Capability cap2(SPHERE, 0.1, 0.2, 0.3, 2.0);
//
//    ASSERT_TRUE(SPHERE == cap2._type);
//    ASSERT_TRUE(0.1 == cap2._phi);
//    ASSERT_TRUE(0.2 == cap2._theta);
//    ASSERT_TRUE(0.3 == cap2._halfOpeningAngle);
//    ASSERT_TRUE(2.0 == cap2._shapeFitError);
//
//    // test if shape fit error is correctly initialized to zero
//    Capability cap3(CONE, 10.0, 10.0, 10.0);
//    ASSERT_TRUE(0.0 == cap3._shapeFitError);
}

//TEST(Capability, equalityOperators)
//{
//    // test equality operators with various kinds of data
//    Capability cap1(SPHERE, 0.1, 0.2, 0.3);
//    Capability cap2(SPHERE, 0.1, 0.2, 0.3);
//    Capability cap3(SPHERE, 0.0, 0.1, 0.2);
//    Capability cap4(CONE, 0.1, 0.2, 0.3);
//    Capability cap5(CONE, 0.1, 0.2, 0.4);
//    // test for shape fit error
//    Capability cap6(CONE, 0.1, 0.2, 0.4, 2.0);
//
//    ASSERT_TRUE(cap1 == cap2);
//    ASSERT_FALSE(cap1 == cap3);
//    ASSERT_FALSE(cap1 == cap4);
//    ASSERT_FALSE(cap4 == cap5);
//    ASSERT_FALSE(cap5 == cap6);
//
//    ASSERT_FALSE(cap1 != cap2);
//    ASSERT_TRUE(cap1 != cap3);
//    ASSERT_TRUE(cap1 != cap4);
//    ASSERT_TRUE(cap4 != cap5);
//    ASSERT_TRUE(cap5 != cap6);
//}



//TEST(CapabilityOcTreeNode, constructor)
//{
//    // check if constructors do their job
//    Capability emptyCap(EMPTY, 0.0, 0.0, 0.0);
//    Capability sphereCap(SPHERE, 0.1, 0.2, 0.3);
//
//
//    CapabilityOcTreeNode node1;
//    CapabilityOcTreeNode node2(sphereCap);
//    CapabilityOcTreeNode node3(node2);
//
//    ASSERT_TRUE(emptyCap == node1.getCapability());
//    ASSERT_TRUE(sphereCap == node2.getCapability());
//    ASSERT_TRUE(sphereCap == node3.getCapability());
//}

//TEST(CapabilityOcTreeNode, equalityOperator)
//{
//    // test operator== with various kinds of data
//    CapabilityOcTreeNode node1(Capability(CYLINDER_1, 0.2, 0.3, 0.1));
//    CapabilityOcTreeNode node2(Capability(CYLINDER_1, 0.2, 0.3, 0.1));
//    CapabilityOcTreeNode node3(Capability(CYLINDER_2, 0.3, 0.1, 0.2));
//
//    ASSERT_TRUE(node1 == node2);
//    ASSERT_FALSE(node1 == node3);
//}

//TEST(CapabilityOcTreeNode, children)
//{
//    CapabilityOcTreeNode node;
//
//    // TODO: how to check not yet set children (assert(children[i] != NULL) lets the test fail)
//    for (unsigned int i = 0; i < 8; ++i)
//    {
//        //ASSERT_EQ(static_cast<CapabilityOcTreeNode*>(NULL), node.getChild(i));
//    }
//
//    node.createChild(1);
//
//    //ASSERT_EQ(static_cast<CapabilityOcTreeNode*>(NULL), node.getChild(0));
//    ASSERT_NE(static_cast<CapabilityOcTreeNode*>(NULL), node.getChild(1));
//    for (unsigned int i = 2; i < 8; ++i)
//    {
//        //ASSERT_EQ(static_cast<CapabilityOcTreeNode*>(NULL), node.getChild(i));
//    }
//}

//TEST(CapabilityOcTreeNode, set_getCapability)
//{
//    // test set and get functions of CapabilityOcTreeNode
//    Capability cap(CYLINDER_2, 0.4, 0.0, 1.2);
//    CapabilityOcTreeNode node1;
//    CapabilityOcTreeNode node2;
//
//    node1.setCapability(cap);
//    node2.setCapability(CYLINDER_2, 0.4, 0.0, 1.2);
//
//    ASSERT_TRUE(cap == node1.getCapability());
//    ASSERT_TRUE(cap == node2.getCapability());
//}


//TEST(InverseCapabilityOcTree, constructor)
//{
//    // does the constructor construct a CapabilityOcTree?
//    CapabilityOcTree tree(0.1);
//    ASSERT_EQ("CapabilityOcTree", tree.getTreeType());
//}

//TEST(CapabilityOcTree, set_getNodeCapability)
//{
//    // test set and get functions of CapabilityOcTree
//    Capability emptyCap;
//    Capability cap1(SPHERE, 0.1, 0.1, 0.1);
//    Capability cap2(CONE, 0.2, 0.2, 0.2);
//    Capability cap3(CYLINDER_1, 0.3, 0.3, 0.3);
//    Capability cap4(CYLINDER_2, 0.4, 0.4, 0.4);
//
//    CapabilityOcTreeNode* testNode = NULL;
//
//    CapabilityOcTree tree(0.1);
//    testNode = tree.setNodeCapability(1.0, 1.0, 1.0, SPHERE, 0.1, 0.1, 0.1);  // == cap1
//    ASSERT_FALSE(testNode == NULL);
//
//    tree.setNodeCapability(1.0, 1.0, 2.0, Capability(CONE, 0.2, 0.2, 0.2));  // == cap2
//    ASSERT_FALSE(testNode == NULL);
//
//    tree.setNodeCapability(1.2, 1.0, 1.0, cap3);
//    ASSERT_FALSE(testNode == NULL);
//
//    tree.setNodeCapability(1.0, 1.2, 1.0, cap4);
//    ASSERT_FALSE(testNode == NULL);
//
//    tree.setNodeCapability(1.0, 2.0, 1.0, cap4);
//
//    // test if all capabilities are inserted correctly
//    ASSERT_TRUE(cap1 == tree.getNodeCapability(1.0, 1.0, 1.0));
//    ASSERT_TRUE(cap2 == tree.getNodeCapability(1.0, 1.0, 2.0));
//    ASSERT_TRUE(cap3 == tree.getNodeCapability(1.2, 1.0, 1.0));
//    ASSERT_TRUE(cap4 == tree.getNodeCapability(1.0, 1.2, 1.0));
//    ASSERT_TRUE(cap4 == tree.getNodeCapability(1.0, 2.0, 1.0));
//
//    // replace cap3 with cap4
//    tree.setNodeCapability(1.2, 1.0, 1.0, cap4);
//
//    ASSERT_TRUE(cap4 == tree.getNodeCapability(1.2, 1.0, 1.0));
//
//    // test for not inserted nodes
//    ASSERT_TRUE(emptyCap == tree.getNodeCapability(2.0, 2.0, 2.0));
//}

//TEST(CapabilityOcTree, read_writeBinary)
//{
//    CapabilityOcTree tree(0.1);
//    tree.setNodeCapability(1.0, 1.0, 1.0, EMPTY, 0.0, 0.0, 0.0);
//    tree.setNodeCapability(1.2, 1.0, 1.0, SPHERE, 20.0, 20.0, 10.0);
//    tree.setNodeCapability(1.2, 1.2, 1.0, CONE, 20.0, 20.0, 10.0);
//    tree.setNodeCapability(2.0, 1.0, 1.0, CYLINDER_1, 20.0, 20.0, 10.0);
//    tree.setNodeCapability(2.0, 2.0, 1.0, CYLINDER_2, 20.0, 20.0, 10.0);
//
//    tree.setGroupName("group");
//    tree.setBaseName("base");
//    tree.setTipName("tip");
//
//    tree.writeFile("./test_tree.cpm");
//
//    CapabilityOcTree* tree2 = CapabilityOcTree::readFile("./test_tree.cpm");
//
//    remove("./test_tree.cpm");
//
//    ASSERT_TRUE(tree.getNodeCapability(1.0, 1.0, 1.0) == tree2->getNodeCapability(1.0, 1.0, 1.0));
//    ASSERT_TRUE(tree.getNodeCapability(1.2, 1.0, 1.0) == tree2->getNodeCapability(1.2, 1.0, 1.0));
//    ASSERT_TRUE(tree.getNodeCapability(1.2, 1.2, 1.0) == tree2->getNodeCapability(1.2, 1.2, 1.0));
//    ASSERT_TRUE(tree.getNodeCapability(2.0, 1.0, 1.0) == tree2->getNodeCapability(2.0, 1.0, 1.0));
//    ASSERT_TRUE(tree.getNodeCapability(2.0, 2.0, 1.0) == tree2->getNodeCapability(2.0, 2.0, 1.0));
//
//    ASSERT_EQ(tree.getGroupName(), tree2->getGroupName());
//    ASSERT_EQ(tree.getBaseName(), tree2->getBaseName());
//    ASSERT_EQ(tree.getTipName(), tree2->getTipName());
//
//    delete tree2;
//
//    // test if empty, whitespaced or weird group, base and tip names are written and read
//    tree.setGroupName(" /~#_*+:|<> ");
//    tree.setBaseName(" ");
//    tree.setTipName("");
//
//    tree.writeFile("./test_tree.cpm");
//
//    tree2 = CapabilityOcTree::readFile("./test_tree.cpm");
//
//    remove("./test_tree.cpm");
//
//    ASSERT_EQ(tree.getGroupName(), tree2->getGroupName());
//    ASSERT_EQ(tree.getBaseName(), tree2->getBaseName());
//    ASSERT_EQ(tree.getTipName(), tree2->getTipName());
//
//    delete tree2;
//}



int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
