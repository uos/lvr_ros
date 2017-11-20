#include "lvr_ros/FileObserver.hpp"

#include <iostream>
#include <exception>

void FileObserver::observe()
{
    int fd = inotify_init();
    int descriptor = inotify_add_watch(fd,
                                       this->path.string().c_str(),
                                       IN_MODIFY | IN_CREATE | IN_DELETE);
    if (descriptor < 0)
    {
        this->error = true;
        return;
    } else
    {
        while (true)
        {
            char buf[1024];
            int len                     = read(fd, buf, 1024);
            struct inotify_event* event = (struct inotify_event *) buf;
            FSEvent fsevent;
            fsevent.file = this->path / bfs::path(event->name);

            std::lock_guard<std::mutex> guard(mutex);
            if (event->mask & IN_MODIFY and this->modified_cb)
            {
                fsevent.type = MODIFIED;
                this->modified_cb(fsevent);
            }
            if (event->mask & IN_CREATE and this->created_cb)
            {
                fsevent.type = CREATED;
                this->created_cb(fsevent);
            }
            if (event->mask & IN_DELETE and this->deleted_cb)
            {
                fsevent.type = DELETED;
                this->deleted_cb(fsevent);
            }
        }
    }
}

void FileObserver::on_created(EventCallback callback)
{
    std::lock_guard<std::mutex> guard(mutex);
    this->created_cb = callback;
}

void FileObserver::on_modified(EventCallback callback)
{
    std::lock_guard<std::mutex> guard(mutex);
    this->modified_cb = callback;
}

void FileObserver::on_deleted(EventCallback callback)
{
    std::lock_guard<std::mutex> guard(mutex);
    this->deleted_cb = callback;
}

bool FileObserver::good()
{
    return !this->error;
}
