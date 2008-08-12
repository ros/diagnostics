///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 16 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "gen_hokuyo_tester.h"

///////////////////////////////////////////////////////////////////////////

GenHokuyoTester::GenHokuyoTester( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	this->SetMinSize( wxSize( 640,480 ) );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxHORIZONTAL );
	
	visPanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	bSizer1->Add( visPanel, 1, wxALL|wxEXPAND, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer31;
	bSizer31 = new wxBoxSizer( wxHORIZONTAL );
	
	testButton = new wxButton( this, wxID_ANY, wxT("Test"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer31->Add( testButton, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxEXPAND, 5 );
	
	bSizer4->Add( bSizer31, 0, wxEXPAND|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	logText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE );
	bSizer4->Add( logText, 1, wxALL|wxEXPAND, 5 );
	
	bSizer1->Add( bSizer4, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer1 );
	this->Layout();
	bSizer1->Fit( this );
	
	// Connect Events
	testButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenHokuyoTester::OnTest ), NULL, this );
}

GenHokuyoTester::~GenHokuyoTester()
{
	// Disconnect Events
	testButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenHokuyoTester::OnTest ), NULL, this );
}
