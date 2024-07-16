#ifndef OBSERVER_H
#define OBSERVER_H

#define MAX_OBSERVERS 10

template <typename T>
class Observer {
    private:
        T state;

    public:
        void update(T newState);
        T getState();
};


template <typename T>
void Observer<T>::update(T newState)
{
    state = newState;
}

template <typename T>
T Observer<T>::getState()
{
    return state;
}

#endif
