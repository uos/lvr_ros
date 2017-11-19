#ifndef FILEOBSERVER_H
#define FILEOBSERVER_H

#include <sys/inotify.h>
#include <boost/filesystem.hpp>
#include <thread>

namespace bfs = boost::filesystem;

typedef enum
{
    CREATED = 0,
    MODIFIED = 1,
    DELETED = 2
} FSEventType;

struct FSEvent
{
    const bfs::path file;
    const FSEventType type;
};

// Function type for callback
typedef void (*EventCallback)(const FSEvent&);

class FileObserver
{
    private:
        bfs::path path;
        EventCallback created_cb;
        EventCallback modfied_cb;
        EventCallback deleted_cb;
        std::thread runner;

    public:
		virtual ~FileObserver()
        {
            // TODO: Stop runner thread?
        }

        FileObserver(const bfs::path& path) : path(path), created_cb(nullptr),
                                               modfied_cb(nullptr), deleted_cb(nullptr)
        {
            this->runner = std::thread(boost::bind(&FileObserver::observe, this));
            runner.detach();
        }

        void observe()
        {
            int fd = inotify_init();
            int descriptor = inotify_add_watch(fd, this->path.string().c_str(), IN_MODIFY | IN_CREATE | IN_DELETE);
            if (descriptor < 0)
            {
                // TODO: Error
            } else
            {
                while (true)
                {
                    char buf[1024];
                    int len;
                    len = read(fd, buf, 1024);
					struct inotify_event* event = (struct inotify_event *) buf;
                    if (event->mask & IN_MODIFY)
                    {
                        std::cout << std::string(event->name) << " was modified." << std::endl;
                    }
                    if (event->mask & IN_CREATE)
                    {
                        std::cout << std::string(event->name) << " was created." << std::endl;
                    }
                    if (event->mask & IN_DELETE)
                    {
                        std::cout << std::string(event->name) << " was deleted." << std::endl;
                    }
				}
            }
        }
        void on_created(EventCallback& callback)
        {
            this->created_cb = callback;
        }

        void on_modified(EventCallback& callback)
        {
            this->modfied_cb = callback;
        }

        void on_deleted(EventCallback& callback)
        {
            this->deleted_cb = callback;
        }
};

#endif /* FILEOBSERVER_H */
