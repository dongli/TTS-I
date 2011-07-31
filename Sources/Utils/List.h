#ifndef _List_h_
#define _List_h_

#include "ReportMacros.h"
#include <algorithm>
#include <queue>
#include <string>

using std::queue;
using std::string;

template <class T>
class ListElement
{
public:
    enum EndTag {
        Head = 0, Tail = 1, Body = 2, Null = 3
    };

    ListElement();
    virtual ~ListElement();

    virtual void reinit();

    void setID(int ID);
    virtual int getID() const;

    T *prev, *next;

    EndTag endTag;

private:
    int ID;
};

template <class T>
class List
{
public:
    List(int initPoolSize = 10, int incrementSize = 10);
    virtual ~List();

    void setName(char const *);

    void create(int numElem);

    void append(T *);
    void append(T **);
    void append();

    void insert(T *elem1, T **elem);
    void insert(T **elem, T *elem1);

    void ring();
    bool isRing();

    void swap(List<T> *);

    void remove(T *);
    void erase(T *);

    void recycle();
    void destroy();

    void reindex();
    void shift(int);

    int size() const;

    T *front() const;
    T *back() const;
    T *at(int) const;

protected:
    string name;

    // Shared by constructor and create member function
    void reinit(int initPoolSize = 10, int incrementSize = 10);

    // Object pool operations
    void initPool(int);
    void increasePool(int);
    T *getFreeElem();

    // Object pool variables
    int poolSize;
    int incrementSize;
    T *poolHead, *poolTail;
    queue<T *> freeElem;

    // List contents
    int numElem;
    T *head, *tail;

    // Counters
    int IDCounter; // For setting the element ID

    // Workflow indicators:
    bool isDestroyed;
};

#include "List.cpp"

#endif
