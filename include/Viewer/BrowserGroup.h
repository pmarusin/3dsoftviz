#ifndef BROWSERGROUP_H
#define BROWSERGROUP_H

#include <osg/ref_ptr>
#include <osg/AutoTransform>
#include <osg/Group>
#include <QList>

#include "OsgQtBrowser/QWebViewImage.h"
#include "Data/Node.h"
#include "LuaGraph/LuaNode.h"
#include "LuaGraph/LuaGraphTreeModel.h"

namespace Vwr {
/**
	*  \class BrowserGroup
	*  \brief
	*  \author Michael Gloger
	*  \date 16. 11. 2014
	*/
class BrowserGroup
{
public:
	/**
		*  \fn public constructor  BrowserGroup()
		*  \brief Creates browser group
		*/
	BrowserGroup();

	/**
		*  \fn public destructor  ~BrowserGroup
		*  \brief destructor
		*/
	~BrowserGroup( void );

	/**
		*  \fn public  setSelectedNodes
		*  \brief Adds only nodes not previously added to selectedNodesModels map
		*  \param  selected
		*/
	void setSelectedNodes( QLinkedList<osg::ref_ptr<Data::Node> >* selected );

	/**
		*  \fn public  setBrowsersGrouping
		*  \brief Changes browsersGrouping value
		*/
	void setBrowsersGrouping( bool browsersGrouping );

	/**
		*  \fn public  updateBrowsers
		*  \brief Animates browsers pop out interpolation
		*/
	void updateBrowsers();

	/**
		*  \fn public  getGroup
		*  \brief Returns wrapped browsers group
		*  \return osg::ref_ptr browsers group
		*/
	osg::ref_ptr<osg::Group> getGroup()
	{
		return group;
	}

	/**
		*  \fn public  getSelectedNodesModels
		*  \return Selected nodes models map
		*/
	inline QMap<qlonglong, Lua::LuaGraphTreeModel* >* getSelectedNodesModels()
	{
		return selectedNodesModels;
	}

private:

	/**
		*  \fn private  addBrowser
		*  \brief Adds browser at position, passes supplied list of model data objects to js variable qData and calls js function qDataReady.
		*  \param  position
		*  \param  *models
		*/
	void addBrowser( osg::Vec3 position, QList<Lua::LuaGraphTreeModel*>* models );

	/**
		*  \fn private  initBrowsers
		*  \brief Initializes sepparate browser for all selected nodes models
		*/
	void initBrowsers();

	/**
		*  \fn private  initGroupedBrowser
		*  \brief Initializes one grouped browser for all selected nodes models
		*/
	void initGroupedBrowser();

	/**
		*  \fn private  clearBrowsers
		*  \brief Removes all browsers from group
		*/
	void clearBrowsers();

	/**
		*  \fn private  clearModels
		*  \brief Removes all model
		*/
	void clearModels();

	/**
		*  osg::ref_ptr group
		*  \brief browsers group
		*/
	osg::ref_ptr<osg::Group> group;

	/**
		*  bool browsersGrouping
		*  \brief bool value indicating if browsers are grouping into one
		*/
	bool browsersGrouping;

	/**
		*  \brief List of browsers transforms
		*/
	QList<osg::ref_ptr<osg::AutoTransform> >* browsersTransforms;

	/**
		*  \brief Map of Data::Node ids to Data::Node containing all selected nodes
		*/
	QMap<qlonglong, osg::ref_ptr<Data::Node> >* selectedNodes;

	/**
		*  \brief Map of Data::Node ids to Lua::LuaGraphTreeModel(s) containing all selected nodes
		*/
	QMap<qlonglong, Lua::LuaGraphTreeModel*>* selectedNodesModels;

	/**
		*  \fn public  interpolate
		*  \brief Calculates interpolation function
		*  \param  currentFrame
		*  \param  endFrame
		*  \param  startValue
		*  \param  endValue
		*  \return Caculated interpolation value
		*/
	double interpolate( long currentFrame, long endFrame, double startValue, double endValue );
};
}

#endif // BROWSERGROUP_H
