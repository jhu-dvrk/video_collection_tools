// Author(s): Anton Deguet
// Copyright 2025 Johns Hopkins University

#pragma once

#include <gtk/gtk.h>
#include <iostream>
#include <streambuf>
#include <string>

class GtkStreamBuf : public std::streambuf {
public:
  GtkStreamBuf() : m_buffer(nullptr), m_view(nullptr) {}

  void set_widgets(GtkTextView *view);

protected:
  virtual std::streamsize xsputn(const char *s, std::streamsize n) override;
  virtual int_type overflow(int_type c) override;

private:
  GtkTextBuffer *m_buffer;
  GtkTextView *m_view;
};
