// this example demonstrates the beviorial difference between
// Sequence, MemSequence, Selector, StatefulSelector
//
// g++ -std=c++11 BrainTree_demo.cpp && ./a.out
#include <iostream>
using namespace std;
#include "../BrainTree.h"

#define STR(var) #var

enum BoardItem {
    MSG,
};

class SuccessAction : public BrainTree::Node {
public:
    SuccessAction(int i) {
         id = i;
    }
    Status update() override {
        cout << "  success" << id << " action" << endl;
        sprintf(buffer, "    message from success");
        assert(blackboard != nullptr && "The Blackboard is empty!");
        blackboard->setString(STR(MSG), buffer);
        return Status::Success;
    }
private:
    char buffer[128]; 
    int id;
};

class FailAction : public BrainTree::Node {
public:
    Status update() override {
        cout << "  fail action" << endl;
        assert(blackboard != nullptr && "The Blackboard is empty!");
        cout << blackboard->getString(STR(MSG)) << endl;
        return Status::Failure;
    }
};

void CreatingBehaviorTreeManually() {
    cout << "*** Creating BehaviorTree Manually" << endl;
    cout << "Sequence: success1 -> fail -> success2, three times" << endl;
    BrainTree::BehaviorTree tree;
    auto sequence = new BrainTree::Sequence();
    auto success1 = new SuccessAction(1);
    auto fail = new FailAction();
    auto success2 = new SuccessAction(2);
    success1->setBlackboard(tree.getBlackboard());
    fail->setBlackboard(tree.getBlackboard());
    success2->setBlackboard(tree.getBlackboard());
    sequence->addChild(success1);
    sequence->addChild(fail);
    sequence->addChild(success2);
    tree.setRoot(sequence);
    cout << " first update()" << endl;
    tree.update();
    cout << " second update()" << endl;
    tree.update();
    cout << " third update()" << endl;
    tree.update();
}

void CreatingBehaviorTreeUsingBuilders() {
    cout << "*** Creating BehaviorTree Using Builders" << endl;
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
    CreatingBehaviorTreeManually();
    CreatingBehaviorTreeUsingBuilders();
    return 0;
}