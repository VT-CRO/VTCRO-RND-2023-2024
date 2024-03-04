#ifndef SUBJECT_H
#define SUBJECT_H

#include <Observer.hpp>

template <typename T>
class Subject {
    private:
        Observer<T> observers[MAX_OBSERVERS];
        T _state;
        int _num_observers;

    public:
        Subject();
        Subject(T initial_state);
        void set_state(T state);
        bool attachObserver(Observer<T> obs);
        void notifyObservers();

    friend Observer<T>;
};

template <typename T>
Subject<T>::Subject() : _num_observers(0) {}

template <typename T>
Subject<T>::Subject(T initial_state)
{
    _state = initial_state;
}

template <typename T>
void Subject<T>::set_state(T state)
{
    _state = state;
}

template <typename T>
bool Subject<T>::attachObserver(Observer<T> obs)
{
    if (_num_observers < MAX_OBSERVERS)
    {
        observers[_num_observers++] = obs;
        return true;
    }
    else return false;
}

template <typename T>
void Subject<T>::notifyObservers()
{
    for (int i = 0; i < _num_observers; ++i)
    {
        observers[i].update(_state);
    }
}

#endif