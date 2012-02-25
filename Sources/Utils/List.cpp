template <class T>
ListElement<T>::ListElement()
{
    ID = -1;
    prev = NULL;
    next = NULL;
}

template <class T>
ListElement<T>::~ListElement()
{
}

template <class T>
void ListElement<T>::reinit()
{
    static int warningCounter = 0;
    if (warningCounter < 10) {
        REPORT_WARNING("reinit function should be overridden by the subclass.")
        ++warningCounter;
    } else if (warningCounter == 5) {
        REPORT_WARNING("More of this warning has been suppressed.")
        ++warningCounter;
    }
}

template <class T>
void ListElement<T>::clean()
{
}

template <class T>
void ListElement<T>::setID(int ID)
{
    this->ID = ID;
}

template <class T>
int ListElement<T>::getID() const
{
    return this->ID;
}

// -----------------------------------------------------------------------------
template <class T>
List<T>::List(int initPoolSize, int incrementSize)
{
    name = "A double linked list";
    reinit(initPoolSize, incrementSize);
}

template <class T>
List<T>::~List()
{
    if (isDestroyed == false)
        destroy();
}

template <class T>
void List<T>::setName(char const *name)
{
    this->name = name;
}

template <class T>
void List<T>::reinit(int initPoolSize, int incrementSize)
{
    // Create a pool with the given size
    poolSize = 0;
    initPool(initPoolSize);
    this->incrementSize = incrementSize;
    // Initiate the list
    head = NULL;
    tail = NULL;
    numElem = 0;
    // Zero counters
    IDCounter = 0;
    // Initiate workflow indicators
    isDestroyed = false;
    isRinged = false;
}

template <class T>
void List<T>::create(int size)
{
    if (isDestroyed == true) {
        reinit(size, incrementSize);
    }
    for (int i = 0; i < size; ++i)
        append();
}

template <class T>
void List<T>::append(T **elem)
{
    append();
    *elem = tail;
}

template <class T>
void List<T>::append()
{
    ++numElem;
    if (numElem != 1) {
        T *elem = getFreeElem();
        elem->prev = tail;
        tail->next = elem;
        tail = elem;
        if (!isRing()) {
            tail->next = NULL;
            if (tail->prev != head)
                tail->prev->endTag = ListElement<T>::Body;
            tail->endTag = ListElement<T>::Tail;
        } else {
            tail->next = head;
            head->prev = tail;
            tail->endTag = ListElement<T>::Body;
        }
    } else {
        head = getFreeElem();
        head->prev = NULL;
        head->endTag = ListElement<T>::Head;
        tail = head;
        tail->next = NULL;
    }
}

template <class T>
void List<T>::insert(T* elem1, T** elem)
{
    ++numElem;
    *elem = getFreeElem();
    if (!isRing()) {
        if (elem1 == tail) {
            (*elem)->next = NULL;
            (*elem)->prev = elem1;
            (*elem)->endTag = ListElement<T>::Tail;
            tail = *elem;
            elem1->next = *elem;
            if (elem1 != head)
                elem1->endTag = ListElement<T>::Body;
            else
                elem1->endTag = ListElement<T>::Head;
        } else {
            T *elem2 = elem1->next;
            elem2->prev = *elem;
            (*elem)->next = elem2;
            (*elem)->prev = elem1;
            (*elem)->endTag = ListElement<T>::Body;
            elem1->next = *elem;
        }
    } else {
        T *elem2 = elem1->next;
        elem2->prev = *elem;
        (*elem)->next = elem2;
        (*elem)->prev = elem1;
        (*elem)->endTag = ListElement<T>::Body;
        elem1->next = *elem;
        if (elem1 == tail) {
            tail = *elem;
        }
    }
}

template <class T>
void List<T>::insert(T** elem, T* elem1)
{
    ++numElem;
    *elem = getFreeElem();
    if (!isRing()) {
        if (elem1 == head) {
            (*elem)->prev = NULL;
            (*elem)->next = elem1;
            (*elem)->endTag = ListElement<T>::Head;
            head = *elem;
            elem1->prev = *elem;
            if (elem1 != tail)
                elem1->endTag = ListElement<T>::Body;
            else
                elem1->endTag = ListElement<T>::Tail;
        } else {
            T *elem2 = elem1->prev;
            elem2->next = *elem;
            (*elem)->prev = elem2;
            (*elem)->next = elem1;
            (*elem)->endTag = ListElement<T>::Body;
            elem1->prev = *elem;
        }
    } else {
        T *elem2 = elem1->prev;
        elem2->next = *elem;
        (*elem)->prev = elem2;
        (*elem)->next = elem1;
        (*elem)->endTag = ListElement<T>::Body;
        elem1->prev = *elem;
        if (elem1 == head) {
            head = *elem;
        }
    }
}

template <class T>
void List<T>::ring()
{
    head->prev = tail;
    tail->next = head;
    head->endTag = ListElement<T>::Body;
    tail->endTag = ListElement<T>::Body;
    isRinged = true;
}

template <class T>
bool List<T>::isRing()
{
    return isRinged;
}

template <class T>
void List<T>::swap(List<T> *that)
{
    std::swap(this->poolSize, that->poolSize);
    std::swap(this->incrementSize, that->incrementSize);
    std::swap(this->poolHead, that->poolHead);
    std::swap(this->poolTail, that->poolTail);
    std::swap(this->freeElem, that->freeElem);
    std::swap(this->numElem, that->numElem);
    std::swap(this->head, that->head);
    std::swap(this->tail, that->tail);
    std::swap(this->IDCounter, that->IDCounter);
    std::swap(this->isDestroyed, that->isDestroyed);
}

template <class T>
void List<T>::remove(T *elem)
{
    if (elem->endTag == ListElement<T>::Null)
        REPORT_ERROR("Element has already been removed.");
    --numElem;
    if (elem->endTag != ListElement<T>::Head) {
        elem->prev->next = elem->next;
        if (elem == head)
            // The list has been made as a ring
            head = (T *) elem->next;
    } else {
        if (numElem != 0) {
            head = (T *) head->next;
            head->endTag = ListElement<T>::Head;
        } else {
            // There is only one element
            head = NULL;
            tail = NULL;
            elem->endTag = ListElement<T>::Null;
            freeElem.push(elem);
            return;
        }
    }
    if (elem->endTag != ListElement<T>::Tail) {
        elem->next->prev = elem->prev;
        if (elem == tail)
            // The list has been made as a ring
            tail = (T *) tail->prev;
    } else {
        tail = (T *) tail->prev;
        if (numElem != 1)
            tail->endTag = ListElement<T>::Tail;
    }
    elem->endTag = ListElement<T>::Null;
    elem->clean();
    freeElem.push(elem);
}

template <class T>
void List<T>::erase(T *elem)
{
    --numElem;
    --poolSize;
    if (elem->endTag != ListElement<T>::Head) {
        ((T *) elem->prev)->next = elem->next;
        if (elem == head)
            // The list has been made as a ring
            head = (T *) elem->next;
    } else {
        if (numElem != 0) {
            head = (T *) head->next;
            head->endTag = ListElement<T>::Head;
        } else {
            // There is only one element
            head = NULL;
            tail = NULL;
            elem->endTag = ListElement<T>::Null;
            freeElem.push(elem);
            return;
        }
    }
    if (elem->endTag != ListElement<T>::Tail) {
        elem->next->prev = elem->prev;
        if (elem == tail)
            // The list has been made as a ring
            tail = (T *) tail->prev;
    } else {
        tail = (T *) tail->prev;
        if (numElem != 1)
            tail->endTag = ListElement<T>::Tail; // This is buggy! 2011-2-28
    }
    delete elem;
}

template <class T>
inline void List<T>::recycle()
{
    // Recycle the used elements
    for (int i = 0; i < numElem; ++i) {
        freeElem.push(head);
        head = (T *) head->next;
    }
    head = NULL;
    tail = NULL;
    numElem = 0;
    // Reset some counters
    IDCounter = 0;
}

template <class T>
void List<T>::destroy()
{
    if (isDestroyed)
        return;
    // Destroy the list contents
    T *temp;
    for (int i = 0; i < numElem; ++i) {
        temp = (T *) head->next;
        delete head;
        head = temp;
    }
    numElem = 0;
    // Destroy the free elements
    while (!freeElem.empty()) {
        //! \note Reinitialize the element in freeElem queue to ensure the
        //!       deletion of it will not interfere with other objects.
        freeElem.front()->reinit();
        delete freeElem.front();
        freeElem.pop();
    }
    poolSize = 0; // This is buggy! 2011-2-28
    // Destroy the head and tail of the pool
    delete poolHead;
    delete poolTail;
    // Reset some counters
    IDCounter = 0;
    // Set the workflow indicator
    isDestroyed = true;
}

template <class T>
void List<T>::reindex()
{
    T *elem = head;
    for (int i = 0; i < numElem; ++i) {
        elem->setID(i+1);
        elem = (T *) elem->next;
    }
    IDCounter = numElem; // This is buggy! 2011-02-20
}

template <class T>
void List<T>::shift(int count)
{
    if (!isRing())
        REPORT_ERROR("The list has not been made as a ring!");
    if (count > 0) {
        head = at(count);
    } else {
        head = at(numElem-count);
    }
    tail = (T *) head->prev;
}

template <class T>
int List<T>::size() const
{
    return numElem;
}

template <class T>
T *List<T>::front() const
{
    return head;
}

template <class T>
T *List<T>::back() const
{
    return tail;
}

template <class T>
T *List<T>::at(int idx) const
{
    T *elem;
    if (idx > numElem) {
        cout << "Wanted index: " << idx << endl;
        cout << "number of elements: " << numElem << endl;
        REPORT_ERROR("Bad index!")
    }
    if (idx < numElem/2) {
        elem = head;
        for (int i = 0; i < idx; ++i) {
            elem = (T *) elem->next;
        }
    } else {
        elem = tail;
        for (int i = numElem-1; i > idx; --i) {
            elem = (T *) elem->prev;
        }
    }
    return elem;
}

template <class T>
void List<T>::initPool(int size)
{
    // Initiate the head and tail of the pool
    // They are not allowed to be used
    poolHead = new T;
    poolTail = new T;
    poolHead->prev = NULL;
    poolHead->next = poolTail;
    poolTail->prev = poolHead;
    increasePool(size);
}

template <class T>
void List<T>::increasePool(int size)
{
    poolSize += size;
    T *temp = poolTail;
    for (int i = 0; i < size; ++i) {
        temp->endTag = ListElement<T>::Body;
        freeElem.push(temp);
        temp->next = new T;
        temp = (T *) temp->next;
        temp->prev = poolTail;
        poolTail->next = temp;
        poolTail = temp;
    }
    poolTail->next = NULL;
} 

template <class T>
T *List<T>::getFreeElem()
{
    if (freeElem.empty())
        // Reach the tail of the pool
        increasePool(incrementSize);
    T *elem = freeElem.front();
    freeElem.pop();
    elem->setID(++IDCounter);
    elem->reinit(); // This is very buggy! 2010-12-22
    return elem;
}

