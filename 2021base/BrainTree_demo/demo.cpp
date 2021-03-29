// this example demonstrates the beviorial difference between
// Sequence, MemSequence, Selector, StatefulSelector
//
// g++ -std=c++11 demo.cpp && ./a.out
#include <iostream>
using namespace std;
#include "../BrainTree.h"

class SuccessAction : public BrainTree::Leaf {
public:
    SuccessAction(int i) {
         id = i;
    }
    Status update() override {
        cout << "  success" << id << " action" << endl;
        return Node::Status::Success;
    }
private:
    int id;
};

class FailAction : public BrainTree::Leaf {
public:
    Status update() override {
        cout << "  fail action" << endl;
        return Node::Status::Failure;
    }
};

#if 0 // not used
void CreatingBehaviorTreeManually() {
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

void CreatingBehaviorTreeUsingBuilders() {
    cout << "Sequence: success1 -> fail -> success2, three times" << endl;
    auto tree = BrainTree::Builder()
        .composite<BrainTree::Sequence>()
            .leaf<SuccessAction>(1)
            .leaf<FailAction>()
            .leaf<SuccessAction>(2)
        .end()
        .build();
    cout << " first update()" << endl;
    tree->update();
    cout << " second update()" << endl;
    tree->update();
    cout << " third update()" << endl;
    tree->update();

    cout << "MemSequence: success1 -> fail -> success2, three times" << endl;
    tree = BrainTree::Builder()
        .composite<BrainTree::MemSequence>()
            .leaf<SuccessAction>(1)
            .leaf<FailAction>()
            .leaf<SuccessAction>(2)
        .end()
        .build();
    cout << " first update()" << endl;
    tree->update();
    cout << " second update()" << endl;
    tree->update();
    cout << " third update()" << endl;
    tree->update();

    cout << "Selector: fail -> success1 -> success2, three times" << endl;
    tree = BrainTree::Builder()
        .composite<BrainTree::Selector>()
            .leaf<FailAction>()
            .leaf<SuccessAction>(1)
            .leaf<SuccessAction>(2)
        .end()
        .build();
    cout << " first update()" << endl;
    tree->update();
    cout << " second update()" << endl;
    tree->update();
    cout << " third update()" << endl;
    tree->update();

    cout << "StatefulSelector: fail -> success1 -> success2, three times" << endl;
    tree = BrainTree::Builder()
        .composite<BrainTree::StatefulSelector>()
            .leaf<FailAction>()
            .leaf<SuccessAction>(1)
            .leaf<SuccessAction>(2)
        .end()
        .build();
    cout << " first update()" << endl;
    tree->update();
    cout << " second update()" << endl;
    tree->update();
    cout << " third update()" << endl;
    tree->update();

    cout << "Sequence: Succeeder w/ fail child" << endl;
    cout << "-> Inverted Failer w/ success1 child" << endl;
    cout << "-> success2" << endl;
    tree = BrainTree::Builder()
        .composite<BrainTree::Sequence>()
            .decorator<BrainTree::Succeeder>()
                .leaf<FailAction>()
            .end()
            .decorator<BrainTree::Inverter>()
                .decorator<BrainTree::Failer>()
                    .leaf<SuccessAction>(1)
                .end()
            .end()
            .leaf<SuccessAction>(2)
        .end()
        .build();
    tree->update();

    delete tree;
}

int main() {
    //CreatingBehaviorTreeManually();
    CreatingBehaviorTreeUsingBuilders();
    return 0;
}