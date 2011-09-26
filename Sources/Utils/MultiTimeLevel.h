#ifndef _MultiTimeLevel_h_
#define _MultiTimeLevel_h_

enum TimeLevel {
    NewTimeLevel = 0, OldTimeLevel = 1, HalfTimeLevel
};

template <typename T, int TIMELEVEL>
class MultiTimeLevel
{
public:
    MultiTimeLevel() {
        inited = false;
        linked = false;
        mirrored = false;
    }
    virtual ~MultiTimeLevel() {
        if (!mirrored) {
            if (inited || linked)
                for (int i = 1; i < TIMELEVEL; ++i)
                    delete value[i];
            if (inited && !linked)
                delete value[0];
        }
    }

    void init() {
        for (int i = 0; i < TIMELEVEL; ++i)
            value[i] = new T;
        inited = true;
    }

    void link(T &value) {
        this->value[0] = &value;
        for (int i = 1; i < TIMELEVEL; ++i)
            this->value[i] = new T;
        linked = true;
    }

    void mirror(MultiTimeLevel<T, TIMELEVEL> &that) {
        for (int i = 0; i < TIMELEVEL; ++i)
            this->value[i] = that.value[i];
        mirrored = true;
    }

    void setOld(const T &value) {
        *(this->value[1]) = value;
    }

    const T &getOld() const {
        return *(value[1]);
    }

    void setNew(const T &value) {
        *(this->value[0]) = value;
    }

    const T &getNew() const {
        return *(value[0]);
    }

    void set(TimeLevel time, const T &value) {
        *(this->value[time]) = value;
    }

    T get(TimeLevel time) const {
        T res;
        switch (time) {
            case OldTimeLevel:
                res = *(value[1]);
                break;
            case HalfTimeLevel:
                res = (*(value[0])+*(value[1]))*0.5;
                break;
            case NewTimeLevel:
                res = *(value[0]);
                break;
        }
        return res;
    }

    void save() {
        for (int i = TIMELEVEL-1; i > 0; --i)
            *(value[i]) = *(value[i-1]);
    }

    void reset(const T &value) {
        for (int i = 0; i < TIMELEVEL; ++i)
            *(this->value[i]) = value;
    }

    MultiTimeLevel<T, TIMELEVEL>
    &operator=(const MultiTimeLevel<T, TIMELEVEL> &that) {
        if (this != &that)
            for (int i = 0; i < TIMELEVEL; ++i)
                *(this->value[i]) = *(that.value[i]);
        return *this;
    }

    friend bool operator==(const MultiTimeLevel<T, TIMELEVEL> &a,
                           const MultiTimeLevel<T, TIMELEVEL> &b) {
        bool res = true;
        for (int i = 0; i < TIMELEVEL; ++i)
            if (*(a.value[i]) != *(b.value[i])) {
                res = false;
                break;
            }
        return res;
    }

protected:
    bool inited, linked, mirrored;
    T *value[TIMELEVEL];
};

#endif
