#include <chrono>
#include <iostream>
#include "Engine.h"

Engine::Engine(QObject* parent) : QThread(parent)
{
}

void Engine::run()
{
    typedef std::chrono::steady_clock my_clock_t;

    const double min_delta_t = 1.0/30.0;

    std::chrono::time_point<my_clock_t> last_t = my_clock_t::now();

    while( isInterruptionRequested() == false )
    {
        std::chrono::time_point<my_clock_t> current_t = my_clock_t::now();

        auto msecs = std::chrono::duration_cast<std::chrono::milliseconds>(current_t - last_t).count();
        const double delta_t = double(msecs) / 1000.0;

        if( delta_t > min_delta_t )
        {
            m_solver.step(delta_t);
            last_t = current_t;
        }
        else
        {
            QThread::yieldCurrentThread();
        }
    }
}

void Engine::setWorld( std::shared_ptr<World> world )
{
    m_solver.setWorld(world);
}
