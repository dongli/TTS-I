#include "List.h"

BOOST_AUTO_TEST_SUITE(List_Tests)

class IntegerElement : public ListElement<IntegerElement>
{
public:
    IntegerElement() {}
    virtual ~IntegerElement() {}

    void reinit() {}
    void clean() {}

    int value;
};

BOOST_AUTO_TEST_CASE(Move)
{
    List<IntegerElement> list;

    for (int i = 0; i < 5; ++i) {
        list.append();
        list.back()->value = i;
    }

    IntegerElement *elem = list.front();
    for (int i = 0; i < list.size(); ++i) {
        cout << elem->value << endl;;
        elem = elem->next;
    }

    IntegerElement *a, *b;

    a = list.at(0);
    b = list.at(4);

    list.move(&a, b);
    
    elem = list.front();
    for (int i = 0; i < list.size(); ++i) {
        cout << elem->value << endl;;
        elem = elem->next;
    }

    a = list.at(0);
    b = list.at(4);

    list.move(b, &a);

    elem = list.front();
    for (int i = 0; i < list.size(); ++i) {
        cout << elem->value << endl;;
        elem = elem->next;
    }
}

BOOST_AUTO_TEST_SUITE_END()