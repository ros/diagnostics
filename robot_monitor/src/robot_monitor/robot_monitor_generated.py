# -*- coding: utf-8 -*- 

###########################################################################
## Python code generated with wxFormBuilder (version Oct 27 2009)
## http://www.wxformbuilder.org/
##
## PLEASE DO "NOT" EDIT THIS FILE!
###########################################################################

import wx

###########################################################################
## Class MonitorPanelGenerated
###########################################################################

class MonitorPanelGenerated ( wx.Panel ):
	
	def __init__( self, parent ):
		wx.Panel.__init__  ( self, parent, id = wx.ID_ANY, pos = wx.DefaultPosition, size = wx.Size( 500,700 ), style = wx.TAB_TRAVERSAL )
		
		bSizer1 = wx.BoxSizer( wx.VERTICAL )
		
		self.m_splitter2 = wx.SplitterWindow( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.SP_3D )
		self.m_splitter2.SetSashGravity( 0.5 )
		self.m_splitter2.Bind( wx.EVT_IDLE, self.m_splitter2OnIdle )
		self.m_splitter2.SetMinimumPaneSize( 100 )
		
		self.m_panel3 = wx.Panel( self.m_splitter2, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		bSizer5 = wx.BoxSizer( wx.VERTICAL )
		
		sbSizer3 = wx.StaticBoxSizer( wx.StaticBox( self.m_panel3, wx.ID_ANY, u"Errors" ), wx.VERTICAL )
		
		self._error_tree_ctrl = wx.TreeCtrl( self.m_panel3, wx.ID_ANY, wx.DefaultPosition, wx.Size( -1,-1 ), wx.TR_DEFAULT_STYLE|wx.TR_HIDE_ROOT )
		self._error_tree_ctrl.SetMinSize( wx.Size( -1,60 ) )
		
		sbSizer3.Add( self._error_tree_ctrl, 1, wx.ALL|wx.EXPAND, 5 )
		
		bSizer5.Add( sbSizer3, 1, wx.EXPAND, 5 )
		
		sbSizer2 = wx.StaticBoxSizer( wx.StaticBox( self.m_panel3, wx.ID_ANY, u"Warnings" ), wx.VERTICAL )
		
		self._warning_tree_ctrl = wx.TreeCtrl( self.m_panel3, wx.ID_ANY, wx.DefaultPosition, wx.Size( -1,-1 ), wx.TR_DEFAULT_STYLE|wx.TR_HIDE_ROOT )
		self._warning_tree_ctrl.SetMinSize( wx.Size( -1,60 ) )
		
		sbSizer2.Add( self._warning_tree_ctrl, 1, wx.ALL|wx.EXPAND, 5 )
		
		bSizer5.Add( sbSizer2, 1, wx.EXPAND, 5 )
		
		self.m_panel3.SetSizer( bSizer5 )
		self.m_panel3.Layout()
		bSizer5.Fit( self.m_panel3 )
		self.m_panel4 = wx.Panel( self.m_splitter2, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		bSizer4 = wx.BoxSizer( wx.VERTICAL )
		
		sbSizer1 = wx.StaticBoxSizer( wx.StaticBox( self.m_panel4, wx.ID_ANY, u"All" ), wx.VERTICAL )
		
		self._tree_ctrl = wx.TreeCtrl( self.m_panel4, wx.ID_ANY, wx.DefaultPosition, wx.Size( -1,-1 ), wx.TR_DEFAULT_STYLE|wx.TR_HIDE_ROOT )
		self._tree_ctrl.SetMinSize( wx.Size( -1,60 ) )
		
		sbSizer1.Add( self._tree_ctrl, 1, wx.ALL|wx.EXPAND, 5 )
		
		bSizer4.Add( sbSizer1, 1, wx.EXPAND, 5 )
		
		self.m_panel4.SetSizer( bSizer4 )
		self.m_panel4.Layout()
		bSizer4.Fit( self.m_panel4 )
		self.m_splitter2.SplitHorizontally( self.m_panel3, self.m_panel4, 240 )
		bSizer1.Add( self.m_splitter2, 1, wx.EXPAND, 5 )
		
		self._message_status_text = wx.StaticText( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self._message_status_text.Wrap( -1 )
		self._message_status_text.SetFont( wx.Font( wx.NORMAL_FONT.GetPointSize(), 70, 90, 92, False, wx.EmptyString ) )
		self._message_status_text.SetForegroundColour( wx.Colour( 85, 178, 76 ) )
		self._message_status_text.SetToolTipString( u"asdf" )
		
		bSizer1.Add( self._message_status_text, 0, wx.ALL, 5 )
		
		self.SetSizer( bSizer1 )
		self.Layout()
	
	def __del__( self ):
		pass
	
	def m_splitter2OnIdle( self, event ):
		self.m_splitter2.SetSashPosition( 240 )
		self.m_splitter2.Unbind( wx.EVT_IDLE )
	

###########################################################################
## Class MessageTimelineGenerated
###########################################################################

class MessageTimelineGenerated ( wx.Panel ):
	
	def __init__( self, parent ):
		wx.Panel.__init__  ( self, parent, id = wx.ID_ANY, pos = wx.DefaultPosition, size = wx.Size( 503,63 ), style = wx.TAB_TRAVERSAL )
		
		bSizer4 = wx.BoxSizer( wx.HORIZONTAL )
		
		self.m_slider1 = wx.Slider( self, wx.ID_ANY, 2, 1, 2, wx.DefaultPosition, wx.DefaultSize, wx.SL_AUTOTICKS|wx.SL_HORIZONTAL|wx.SL_LABELS|wx.SL_TOP )
		bSizer4.Add( self.m_slider1, 1, wx.ALL, 5 )
		
		self._pause_button = wx.ToggleButton( self, wx.ID_ANY, u"Pause", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer4.Add( self._pause_button, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 5 )
		
		self.SetSizer( bSizer4 )
		self.Layout()
	
	def __del__( self ):
		pass
	

