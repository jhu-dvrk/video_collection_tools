#include "gtk_stream_buf.h"
#include <unistd.h>

void GtkStreamBuf::set_widgets(GtkTextView* view) {
    m_view = view;
    if (view) {
        m_buffer = gtk_text_view_get_buffer(view);
    } else {
        m_buffer = nullptr;
    }
}

std::streamsize GtkStreamBuf::xsputn(const char* s, std::streamsize n) {
    if (!m_buffer) {
        write(1, s, n);
        return n;
    }
    
    std::string text(s, n);
    
    struct LogData {
        GtkTextBuffer* buffer;
        GtkTextView* view;
        std::string text;
    };
    
    LogData* data = new LogData{m_buffer, m_view, text};
    
    g_idle_add([](gpointer user_data) -> gboolean {
        LogData* d = static_cast<LogData*>(user_data);
        
        if (GTK_IS_TEXT_BUFFER(d->buffer)) {
            GtkTextIter end;
            gtk_text_buffer_get_end_iter(d->buffer, &end);
            gtk_text_buffer_insert(d->buffer, &end, d->text.c_str(), -1);
            
            if (GTK_IS_TEXT_VIEW(d->view)) {
                GtkTextMark* mark = gtk_text_buffer_create_mark(d->buffer, nullptr, &end, FALSE);
                gtk_text_view_scroll_to_mark(d->view, mark, 0.0, TRUE, 0.0, 1.0);
                gtk_text_buffer_delete_mark(d->buffer, mark);
            }
        }
        
        delete d;
        return G_SOURCE_REMOVE;
    }, data);
    
    return n;
}

GtkStreamBuf::int_type GtkStreamBuf::overflow(int_type c) {
    if (c != EOF) {
        char ch = static_cast<char>(c);
        xsputn(&ch, 1);
    }
    return c;
}
