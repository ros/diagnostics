/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "hokuyo_tester.h"
#include "urg_laser.h"
#include <wx/dcclient.h>
#include <math.h>
#include <wx/app.h>

#include <wx/msgdlg.h>
#include <wx/textdlg.h>

#include <time.h>
#include <fstream>

DECLARE_EVENT_TYPE(wxSELF_TEST_DONE,-1)
DEFINE_EVENT_TYPE(wxSELF_TEST_DONE)

HokuyoTester::HokuyoTester( wxWindow* parent, ros::node* rosNode_ )
:
  GenHokuyoTester( parent ), rosNode(rosNode_), createdNode(false), view_scale(50), view_x(0), view_y(0)
{
  if (rosNode == NULL)
  {
    int argc = 0;
    ros::init(argc, NULL);
    rosNode = new ros::node("HokuyoTester");
    createdNode = true;
  }

  scan_mutex = new wxMutex();

  log = new wxLogTextCtrl(logText);

  wxLog::SetActiveTarget(log);

  wxBoxSizer* gl_sizer;
  gl_sizer = new wxBoxSizer( wxHORIZONTAL );

  int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0};

  gl = new wxGLCanvas(visPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE, wxT("LaserViewer"), args);

  gl_sizer->Add(gl, 1,  wxALL | wxEXPAND, 0);

  visPanel->SetSizer( gl_sizer );

  m_init = false;
  
  gl->Connect( wxEVT_PAINT, wxPaintEventHandler( HokuyoTester::OnPaint ), NULL, this );
  gl->Connect( wxEVT_ERASE_BACKGROUND, wxEraseEventHandler( HokuyoTester::OnEraseBackground ), NULL, this );
  gl->Connect( wxEVT_SIZE, wxSizeEventHandler( HokuyoTester::OnSize ), NULL, this );
  gl->Connect( wxEVT_MOTION, wxMouseEventHandler( HokuyoTester::OnMouse ), NULL, this );

  Connect( wxSELF_TEST_DONE, wxCommandEventHandler( HokuyoTester::SelfTestDone ), NULL, this );

}

HokuyoTester::~HokuyoTester()
{
  if (createdNode && rosNode)
  {
    ros::fini();
    delete rosNode;
  }
}

// GetValue().mb_str(wxConvUTF8);
//  

void* TestThread::Entry()
{
  wxLogMessage(_T("Conducting self test...\n"));

  if (ros::service::call("urglaser/self_test", parent->req, parent->res))
  {
    wxLogMessage(_T("Self test completed\n") + wxString::FromAscii(parent->res.info.c_str()));
  }
  else
  {
    wxLogMessage(_T("Could not find hokuyo selftest service.  Ros node must not be running."));
    parent->testButton->Enable();
    return NULL;
  }

  wxCommandEvent* e = new wxCommandEvent(wxSELF_TEST_DONE);
  parent->AddPendingEvent(*e);

  return NULL;
}

void HokuyoTester::OnTest( wxCommandEvent& event )
{

  testButton->Disable();

  logText->Clear();
  
  TestThread* t = new TestThread(this);
  t->Create();
  t->Run();

}

void HokuyoTester::SelfTestDone( wxCommandEvent& event )
{
  int answer = -1;

  if (res.passed)
  {
    rosNode->subscribe("scan", readScan, &HokuyoTester::HandleScan, this);
    scanCount = 0;  
    answer = wxMessageBox(_T("Does the data from the Hokuyo look reasonable."), _T("User Confirmation"), wxYES_NO, this);
    rosNode->unsubscribe("scan");

    if (answer == wxYES)
      wxLogMessage(_T("User verified data looks reasonable."));
    else
      wxLogMessage(_T("User specified data looks unreasonable."));
  }

  if (res.id == std::string("H0000000") || res.id == std::string(""))
  {
    wxString id = _T("");

    while (id == _T(""))
      id = wxGetTextFromUser(_T("Hokuyo ID self reporting failed.\nPlease enter The Hokuyo's SN or WGSN.\nWGSN is barcoded on the bottom of the device and starts with 68-.\nHokuyo SN is written on the side of the device and starts with H"), _T("ID"),
                             _T(""), this);
    
    res.id = std::string(id.mb_str(wxConvUTF8));
  }

  time_t t = ::time(NULL);
  struct tm *tms = localtime(&t);
  char datetime[500];
  snprintf(datetime, sizeof(datetime), "%d-%02d-%02d-%02d-%02d-%02d",
           tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
           tms->tm_hour     , tms->tm_min  , tms->tm_sec);


  std::string fname = res.id + std::string("_") + std::string(datetime) + std::string(".test");
  std::ofstream out(fname.c_str());
  if (res.passed)
  {
    out << "Self test: PASSED" << std::endl;
    out << "Info:" << std::endl << res.info;
    out << std::endl << std::endl;

    if (answer == wxYES)
      out << "Data inspection: PASSED" << std::endl;
    else
      out << "Data inspection: FAILED" << std::endl;
  }
  else
  {
    out << "Self test: FAILED" << std::endl;
    out << "Info:" << std::endl << res.info;
  }
  
  testButton->Enable();
}


void HokuyoTester::HandleScan() {
  scan_mutex->Lock();
  dispScan = readScan;
  scan_mutex->Unlock();

  wxPaintEvent* e = new wxPaintEvent();
  gl->AddPendingEvent(*e);
}

void HokuyoTester::InitGL()
{
  gl->SetCurrent();

  int w,h;
  visPanel->GetClientSize(&w, &h);

  glViewport((GLint) 0, (GLint) 0, (GLint) w, (GLint) h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-w/2, w/2, -h/2, h/2, -1, 1);
    glMatrixMode(GL_MODELVIEW);

}

void HokuyoTester::OnSize(wxSizeEvent& event)
{
    // this is also necessary to update the context on some platforms
    gl->OnSize(event);

    // set GL viewport (not called by wxGLCanvas::OnSize on all platforms...)
    int w, h;
    visPanel->GetClientSize(&w, &h);

    gl->SetCurrent();
    //    glViewport((GLint)((w-m)/2), (GLint)((h-m)/2), (GLint) m, (GLint) m);
    glViewport((GLint) 0, (GLint) 0, (GLint) w, (GLint) h);

    InitGL();
}

void HokuyoTester::OnPaint( wxPaintEvent& WXUNUSED(event) )
{
    Render();
}

void HokuyoTester::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
  // Do nothing, to avoid flashing.
}


void HokuyoTester::OnMouse(wxMouseEvent& event)
{
  int x, y;
  event.GetPosition(&x,&y);

  int dx, dy;
  dx = x - last_x;
  dy = y - last_y;

  last_x = x;
  last_y = y;

  if (event.LeftIsDown())
  {
    view_x += dx / view_scale;
    view_y -= dy / view_scale;

    wxPaintEvent* e = new wxPaintEvent();
    gl->AddPendingEvent(*e);
  }
  if (event.RightIsDown())
  {
    view_scale *= 1.0 - dy * 0.01;

    wxPaintEvent* e = new wxPaintEvent();
    gl->AddPendingEvent(*e);
  }
}

void drawrectgrid(float x_distance, float y_distance, float center_x, float center_y, float x_div, float y_div)
{
  float i = 0;

  glPushMatrix();
  glTranslatef(center_x,center_y,0.0);
  glBegin(GL_LINE_LOOP);
  glVertex2f(-x_distance/2, y_distance/2);
  glVertex2f(x_distance/2, y_distance/2);
  glVertex2f(x_distance/2, -y_distance/2);
  glVertex2f(-x_distance/2, -y_distance/2);
  glEnd();


  for(i =-x_distance/2; i<=x_distance/2; i+=x_div)
  {
    glBegin(GL_LINES);
    glVertex2f(i, y_distance/2);
    glVertex2f(i, -y_distance/2);
    glEnd();
  }

  for(i =-y_distance/2; i<=y_distance/2; i+=y_div)
  {
    glBegin(GL_LINES);
    glVertex2f(x_distance/2, i);
    glVertex2f(-x_distance/2, i);
    glEnd();
  }

  glPopMatrix();
}


void HokuyoTester::Render()
{
    wxPaintDC dc(gl);

    gl->SetCurrent();

    // Init OpenGL once, but after SetCurrent
    if (!m_init)
    {
        InitGL();
        m_init = true;
    }

    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glScalef(view_scale, view_scale, view_scale);
    glTranslatef(view_x, view_y, 0);

    glPushMatrix();

    glColor3f(0.9, 0.9, 0.9);
    drawrectgrid(20.0, 20.0, 0.0, 0.0, 1.0, 1.0);

    bool intensity_avail = false;
    double min_intensity = 1e9, max_intensity = -1e9;
    scan_mutex->Lock();
    if (dispScan.get_intensities_size() == dispScan.get_ranges_size())
    {
      for (size_t i = 0; i < dispScan.get_intensities_size(); i++)
      {
        if (dispScan.intensities[i] > max_intensity)
          max_intensity = dispScan.intensities[i];
        if (dispScan.intensities[i] < min_intensity)
          min_intensity = dispScan.intensities[i];
      }
      if (min_intensity == max_intensity)
        min_intensity = max_intensity - 1; // whatever. this won't happen.
      intensity_avail = true;
    }
    glPointSize(4.0);

    for (size_t i = 0; i < dispScan.get_ranges_size(); i++)
    {
      glColor3f(0.1, 0.3, 0.1);
      glBegin(GL_LINES);
      glVertex2f(0,0);
      const double ang = dispScan.angle_min + dispScan.angle_increment * i;
      const double lx = dispScan.ranges[i] * cos(ang);
      const double ly = dispScan.ranges[i] * sin(ang);
      glVertex2f(lx, ly);
      glEnd();
      if (intensity_avail)
      {
        const float inten = (dispScan.intensities[i] - min_intensity) / 
                            (max_intensity - min_intensity);
        glColor3f(inten, 0, 1.0 - inten);
        glBegin(GL_POINTS);
        glVertex2f(lx, ly);
        glEnd();
      }
    }
  scan_mutex->Unlock();

    glPopMatrix();

    glFlush();
    gl->SwapBuffers();
}
