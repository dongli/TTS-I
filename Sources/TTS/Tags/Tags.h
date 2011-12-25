#ifndef Tags_h
#define Tags_h

template <typename TAG, int NUMTAG>
class Tags
{
public:
    Tags() { reset(); }
    virtual ~Tags() {}

    void reset() { for (int i = 0; i < NUMTAG; ++i) tags[i] = false; }

    void set(TAG tag) { tags[tag] = true; }
    void unset(TAG tag) { tags[tag] = false; }
    bool isSet(TAG tag) { return tags[tag]; }

private:
    bool tags[NUMTAG];
};

#endif
