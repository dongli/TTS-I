#ifndef List_h
#define List_h

#include "ReportMacros.hpp"
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
    virtual void clean();

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
    List(int initPoolSize = 10, int incrementSize = 1);
    virtual ~List();

    void setName(char const *);

    void create(int numElem);

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

    void startLoop(T *&);
    bool isLoopEnd(T *);
    void endLoop();
    T *getNextElem();

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

    // Loop control variables
    T *nextElem;
    bool isHeadPassed;

    // Counters
    int IDCounter; // For setting the element ID

    // Workflow indicators:
    bool isDestroyed;
    bool isRinged;
};

#include "List.cpp"

#endif
