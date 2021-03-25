// this example demonstrates the beviorial difference between
// Sequence, MemSequence, Selector, StatefulSelector
//
// g++ -std=c++11 demo.cpp && ./a.out
#include <iostream>
using namespace std;
#include "../BrainTree.h"

class SuccessAction : public BrainTree::Node
{
public:
    Status update() override
    {
        cout << "  success action" << endl;
        return Node::Status::Success;
    }
};

class FailAction : public BrainTree::Node
{
public:
    Status update() override
    {
        cout << "  fail action" << endl;
        return Node::Status::Failure;
    }
};

#if 0 // not used
void CreatingBehaviorTreeManually()
{
    BrainTree::BehaviorTree tree;
    auto sequence = new BrainTree::Selector();
    auto sayHello = new FailAction();
    auto sayHelloAgain = new SuccessAction();
    sequence->addChild(sayHello);
    sequence->addChild(sayHelloAgain);
    tree.setRoot(sequence);
    tree.update();
}
#endif

void CreatingBehaviorTreeUsingBuilders()
{
    cout << "Sequence: success -> fail, two times" << endl;
    auto tree = BrainTree::Builder()
        .composite<BrainTree::MemSequence>()
            .leaf<SuccessAction>()
            .leaf<FailAction>()
        .end()
        .build();
    cout << " first update()" << endl;
    tree->update();
    cout << " second update()" << endl;
    tree->update();
    //tree->update();
}

int main()
{
    //CreatingBehaviorTreeManually();
    CreatingBehaviorTreeUsingBuilders();
    return 0;
}