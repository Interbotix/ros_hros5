#ifndef EMOHEAD_H
#define EMOHEAD_H

#include "CM730.h"

#define ID_EMOHEAD 21	

namespace Robot
{
	class EMOHead
	{
	public:
		enum REGISTERS
		{
		  REG_MODE = 0x20,
		  REG_SETPOS1, //TODO: Left?
		  REG_SETPOS2, //TODO: Right?
		  REG_SETPOS3, //TODO: Brow?
		  REG_SPEED1 = 0x24,
		  REG_SPEED2,
		  REG_SPEED3,
		  REG_BLINK_NOW = 0x27,
		  REG_LOOK_DIR,
		  REG_NUMEYEDOTS_LEFT,
		  REG_NUMEYEDOTS_RIGHT = 0x2A
		};

		enum LOOK_DIRECTION
		{
			LOOK_FORWARD,
			LOOK_LEFT,
			LOOK_RIGHT
		};

		enum MODES
		{
			DEMO,
			COMMAND
		};

		//TODO: Organize these..
		enum EYEDOTS
		{
			DOTS_UP = 0x1F,
			DOTS_DOWN = 0xF8,
			DOTS_WORRIED_RIGHT = 0xF4,
			DOTS_WORRIED_LEFT = 0xE9,
			DOTS_SAD_RIGHT = 0x2F,
			DOTS_SAD_LEFT = 0x97,
			DOTS_PLUS = 0x5A,
			DOTS_MINUS = 0x18,
			DOTS_BAR = 0x42,
			DOTS_STAR = 0xA5,
			DOTS_SHIP = 0xBA,
			DOTS_BACKSLASH = 0x81,
			DOTS_FWDSLASH = 0x24,
			DOTS_FULL = 0xFF
		};

	public:
		EMOHead( CM730 * p_cm730 )
		{
			m_CM730 = p_cm730;
		}

		~EMOHead(){}

		void SetMode( byte p_mode )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_MODE, p_mode, 0);
		}
		
		void SetPositions( byte p_left, byte p_right, byte p_front )
		{
			SetPositionLeft( p_left );
			SetPositionRight( p_right );
			SetPositionFront( p_front );
		}

		void SetSpeeds( byte p_left, byte p_right, byte p_front )
		{
			SetSpeedLeft( p_left );
			SetSpeedRight( p_right );
			SetSpeedFront( p_front );
		}

		void SetPositionLeft( byte p_position )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_SETPOS1, p_position, 0);
		}
		void SetPositionRight( byte p_position )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_SETPOS2, p_position, 0);
		}
		void SetPositionFront( byte p_position )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_SETPOS3, p_position, 0);
		}

		void SetSpeedLeft( byte p_speed )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_SPEED1, p_speed, 0);
		}
		void SetSpeedRight( byte p_speed )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_SPEED2, p_speed, 0);
		}
		void SetSpeedFront( byte p_speed )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_SPEED3, p_speed, 0);
		}

		void BlinkNow( void )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_BLINK_NOW, 1, 0);
		}

		void SetLookDirection( byte p_direction )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_LOOK_DIR, p_direction, 0);
		}

		void SetEyeDots( byte p_mask )
		{
			SetEyeDotsRight( p_mask );
			SetEyeDotsLeft( p_mask );
		}
		void SetEyeDotsRight( byte p_mask )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_NUMEYEDOTS_RIGHT, p_mask, 0);
		}
		void SetEyeDotsLeft( byte p_mask )
		{
			m_CM730->WriteByte(ID_EMOHEAD, REG_NUMEYEDOTS_LEFT, p_mask, 0);
		}
	private:
		CM730 * m_CM730;

		
	};
}

#endif