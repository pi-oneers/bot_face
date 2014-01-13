// This code is a modified version of https://github.com/mifi/jquery-joystick

// http://stackoverflow.com/questions/2162981/jquery-plugin-authoring-set-different-options-for-different-elements
// http://learn.jquery.com/plugins/advanced-plugin-concepts/
(function($) {
	$.fn.joystick = function(val, arg1, arg2) {
		var getValue = function() {
			var $joystick = $(this);
			var $handle = $joystick.children('.handle');

			var x = $handle.position().left / ($joystick.width()-$handle.width());
			var y = $handle.position().top / ($joystick.height()-$handle.height());
			return {x: x, y: 1-y};
		};

		var setValue = function(x, y) {
			var $joystick = $(this);
			var $handle = $joystick.children('.handle');

			var left = x * ($joystick.width()-$handle.width());
			var top = (1-y) * ($joystick.height()-$handle.height());
			$handle.css('left', left);
			$handle.css('top', top);

			$joystick.data('joystick-options').moveEvent.call($joystick[0], getValue.call(this));
		};

		// Init
		if (typeof val === 'object') {
			var options = $.extend({
				xAxis: true,
				yAxis: true,
				xSnap: false,
				ySnap: false,
				moveEvent: function() {},
				endEvent: function() {},
			}, val);

			this.data('joystick-options', options);

			return this.each(function() {
				var $joystick = $(this);
				var $handle = $joystick.children('.handle');

				$handle.css('left', $joystick.width()/2 - $handle.width()/2);
				$handle.css('top', $joystick.height()/2 - $handle.height()/2);

				$joystick.on('touchstart mousedown', function(e) {
					e.preventDefault();
					$joystick.data('clicked', true)
				});

				// TODO Touchcancel
				$joystick.on('touchend mouseup mouseleave', function(e) {
					e.preventDefault();
					$joystick.data('clicked', false);
                    
                     console.log( e.type );
					
					if ($joystick.data('joystick-options').xSnap) {
						setValue.call($joystick[0], 0.5, getValue.call($joystick[0]).y);
					}
					if ($joystick.data('joystick-options').ySnap) {
						setValue.call($joystick[0], getValue.call($joystick[0]).x, 0.5);
					}

					$joystick.data('joystick-options').endEvent.call($joystick[0], getValue.call(this));
				});

				$joystick.on('touchmove mousemove', function(e) {
					e.preventDefault();
                     
                     var pageX;
                     var pageY;
                     if ( e.type == 'mousemove' )
                     {
                         pageX = e.pageX;
                         pageY = e.pageY;
                     }
                     else
                     {
                         var touch = e.originalEvent.targetTouches[0];
                         pageX = touch.pageX;
                         pageY = touch.pageY;
                     }
                     
					var x = pageX - $joystick.offset().left;
					var y = pageY - $joystick.offset().top;

					if ($joystick.data('clicked')) {
						var handleLeft = x - $handle.width()/2;
						var handleTop = y - $handle.height()/2;
						handleLeft = Math.min(handleLeft, $joystick.width() - $handle.width());
						handleLeft = Math.max(handleLeft, 0);
						handleTop = Math.min(handleTop, $joystick.height() - $handle.height());
						handleTop = Math.max(handleTop, 0);

						if ($joystick.data('joystick-options').xAxis) {
							$handle.css('left', handleLeft);
						}
						if ($joystick.data('joystick-options').yAxis) {
							$handle.css('top', handleTop);
						}

						$joystick.data('joystick-options').moveEvent.call($joystick[0], getValue.call(this));
					}
				});
			});
		}
		else if (typeof val === 'string') {
			switch (val.toLowerCase()) {
				case 'value':
				if (arg1 === undefined) {
					return getValue.call(this[0]);
				}
				else {
					return this.each(function() {
						setValue.call(this, arg1, arg2);
					});
				}
			}
		}
	};
}( jQuery ));
