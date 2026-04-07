#include "gpioevent.h"

#include <format>

void GPIOPin::start(int pinNo,
		    int chipNo)
{

#ifdef DEBUG
    std::cerr << "Init" << std::endl;
#endif

    const std::string chipPath = std::format("/dev/gpiochip{}", chipNo);
    const std::string consumername = std::format("gpioconsumer_{}_{}", chipNo, pinNo);

    // Config the pin as input and detecting falling and rising edegs
    gpiod::line_config line_cfg;
    line_cfg.add_line_settings(
			       pinNo,
			       gpiod::line_settings()
			       .set_direction(gpiod::line::direction::INPUT)
			       .set_edge_detection(gpiod::line::edge::BOTH));
    
    chip = std::make_shared<gpiod::chip>(chipPath);
	    
    auto builder = chip->prepare_request();
    builder.set_consumer(consumername);
    builder.set_line_config(line_cfg);
    request = std::make_shared<gpiod::line_request>(builder.do_request());
    
    thr = std::thread(&GPIOPin::worker, this);
}

void GPIOPin::gpioEvent(const gpiod::edge_event &event)
{
    if (eventCallback) eventCallback(event);
}

void GPIOPin::worker()
{
    running = true;
    while (running)
	{
	    // blocking I/O: thread goes to sleep till an event has happened.
	    bool r = request->wait_edge_events(std::chrono::milliseconds(ISR_TIMEOUT_MS));
	    if (r)
		{
		    gpiod::edge_event_buffer buffer;
		    request->read_edge_events(buffer, 1);
		    // callback
		    gpioEvent(buffer.get_event(0));
		}
	    else
		{
#ifdef DEBUG
		    std::cerr << "Timeout" << std::endl;
#endif
		}
	}
    request->release();
    chip->close();
}

void GPIOPin::stop()
{
    running = false;
    if (thr.joinable()) thr.join();
}
