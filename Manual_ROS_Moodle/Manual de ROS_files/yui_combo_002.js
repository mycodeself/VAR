YUI.add('moodle-core-event', function (Y, NAME) {

// This file is part of Moodle - http://moodle.org/
//
// Moodle is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Moodle is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Moodle.  If not, see <http://www.gnu.org/licenses/>.

/**
 * @module moodle-core-event
 */

var LOGNAME = 'moodle-core-event';

/**
 * List of published global JS events in Moodle. This is a collection
 * of global events that can be subscribed to, or fired from any plugin.
 *
 * @namespace M.core
 * @class event
 */
M.core = M.core || {};

M.core.event = {
    /**
     * This event is triggered when a page has added dynamic nodes to a page
     * that should be processed by the filter system. An example is loading
     * user text that could have equations in it. MathJax can typeset the equations
     * but only if it is notified that there are new nodes in the page that need processing.
     * To trigger this event use M.core.Event.fire(M.core.Event.FILTER_CONTENT_UPDATED, {nodes: list});
     *
     * @event "filter-content-updated"
     * @param nodes {Y.NodeList} List of nodes added to the DOM.
     */
    FILTER_CONTENT_UPDATED: "filter-content-updated"
};

M.core.globalEvents = {
    /**
     * This event is triggered when form has an error
     *
     * @event "form_error"
     * @param formid {string} Id of form with error.
     * @param elementid {string} Id of element with error.
     */
    FORM_ERROR: "form_error"
};


var eventDefaultConfig = {
    emitFacade: true,
    defaultFn: function(e) {
        Y.log('Event fired: ' + e.type, 'debug', LOGNAME);
    },
    preventedFn: function(e) {
        Y.log('Event prevented: ' + e.type, 'debug', LOGNAME);
    },
    stoppedFn: function(e) {
        Y.log('Event stopped: ' + e.type, 'debug', LOGNAME);
    }
};

// Publish events with a custom config here.

// Publish all the events with a standard config.
var key;
for (key in M.core.event) {
    if (M.core.event.hasOwnProperty(key)) {
        Y.publish(M.core.event[key], eventDefaultConfig);
    }
}

//Publish all the global events.
for (key in M.core.globalEvents) {
    if (M.core.globalEvents.hasOwnProperty(key)) {
        Y.Global.publish(M.core.globalEvents[key], Y.merge(eventDefaultConfig, {broadcast: 2}));
    }
}


}, '@VERSION@', {"requires": ["event-custom"]});
YUI.add('moodle-filter_mathjaxloader-loader', function (Y, NAME) {

// This file is part of Moodle - http://moodle.org/
//
// Moodle is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Moodle is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Moodle.  If not, see <http://www.gnu.org/licenses/>.

/**
 * Mathjax JS Loader.
 *
 * @package    filter_mathjaxloader
 * @copyright  2014 Damyon Wiese  <damyon@moodle.com>
 * @license    http://www.gnu.org/copyleft/gpl.html GNU GPL v3 or later
 */
M.filter_mathjaxloader = M.filter_mathjaxloader || {

    /**
     * The users current language - this can't be set until MathJax is loaded - so we need to store it.
     * @property _lang
     * @type String
     * @default ''
     * @private
     */
    _lang: '',

    /**
     * Boolean used to prevent configuring MathJax twice.
     * @property _configured
     * @type Boolean
     * @default ''
     * @private
     */
    _configured: false,

    /**
     * Called by the filter when it is active on any page.
     * This does not load MathJAX yet - it addes the configuration to the head incase it gets loaded later.
     * It also subscribes to the filter-content-updated event so MathJax can respond to content loaded by Ajax.
     *
     * @method typeset
     * @param {Object} params List of configuration params containing mathjaxconfig (text) and lang
     */
    configure: function(params) {

        // Add a js configuration object to the head.
        // See "http://docs.mathjax.org/en/latest/dynamic.html#ajax-mathjax"
        var script = document.createElement("script");
        script.type = "text/x-mathjax-config";
        script[(window.opera ? "innerHTML" : "text")] = params.mathjaxconfig;
        document.getElementsByTagName("head")[0].appendChild(script);

        // Save the lang config until MathJax is actually loaded.
        this._lang = params.lang;

        // Listen for events triggered when new text is added to a page that needs
        // processing by a filter.
        Y.on(M.core.event.FILTER_CONTENT_UPDATED, this.contentUpdated, this);
    },

    /**
     * Set the correct language for the MathJax menus. Only do this once.
     *
     * @method setLocale
     * @private
     */
    _setLocale: function() {
        if (!this._configured) {
            var lang = this._lang;
            if (typeof MathJax !== "undefined") {
                MathJax.Hub.Queue(function () {
                    MathJax.Localization.setLocale(lang);
                });
                MathJax.Hub.Configured();
                this._configured = true;
            }
        }
    },

    /**
     * Called by the filter when an equation is found while rendering the page.
     *
     * @method typeset
     */
    typeset: function() {
        if (!this._configured) {
            var self = this;
            Y.use('mathjax', function() {
                self._setLocale();
                Y.all('.filter_mathjaxloader_equation').each(function(node) {
                    if (typeof MathJax !== "undefined") {
                        MathJax.Hub.Queue(["Typeset", MathJax.Hub, node.getDOMNode()]);
                    }
                });
            });
        }
    },

    /**
     * Handle content updated events - typeset the new content.
     * @method contentUpdated
     * @param Y.Event - Custom event with "nodes" indicating the root of the updated nodes.
     */
    contentUpdated: function(event) {
        var self = this;
        Y.use('mathjax', function() {
            self._setLocale();
            event.nodes.each(function (node) {
                node.all('.filter_mathjaxloader_equation').each(function(node) {
                    if (typeof MathJax !== "undefined") {
                        MathJax.Hub.Queue(["Typeset", MathJax.Hub, node.getDOMNode()]);
                    }
                });
            });
        });
    }
};


}, '@VERSION@', {"requires": ["moodle-core-event"]});
