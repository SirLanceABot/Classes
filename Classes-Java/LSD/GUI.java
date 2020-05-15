/**
    LSD Java Port
    
    Copyright (C) 2014  Chris - anfractuosity.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

import javax.swing.*;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.HashSet;
import java.awt.*;
import java.util.Random;

import lsd.*;

public class GUI extends JFrame {

	private static final long serialVersionUID = -4049043147237235392L;

	GUI() {
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		try {
			BufferedImage myPicture = ImageIO.read(new File("piet.jpg"));
			Graphics2D g2d = myPicture.createGraphics();
			int x = myPicture.getWidth();
			int y = myPicture.getHeight();
		
			HashSet<Line> lines = new HashSet<Line>();

			double [] arr = myPicture.getData().getPixels(0,0,x,y,new double[x*y*3]);

			double [] arr2 = new double[x*y];
		
			System.out.println(arr.length);
			int c=0;
			for(int i = 0; i < arr.length-3; i+=3) {
				double B = arr[i];
				double G = arr[i+1];
				double R = arr[i+2];
				double level = R * 0.2126 + G * 0.7152 + B * 0.0722;
				arr2[c++] = level; //0.0 to 255.0
				//System.out.print("  " + level);
			}

			LSD lsd = new LSD();

			double [] out = lsd.lsd(arr2,x,y);

			for(int i = 0; i < lsd.n_out; i++) {
				
				lines.add(new Line(out[7 * i + 0], out[7 * i + 1],
						out[7 * i + 2], out[7 * i + 3]));
			}

    		final BasicStroke dashed = // "lineWidth","endCap","lineJoin","miterLimit","dashArray","dashPhase"
				new BasicStroke(
					5.0f, //line width
					BasicStroke.CAP_BUTT,
					BasicStroke.JOIN_MITER,
					10.0f,
					new float[] {10.f, 5.f}, // dash segments' lengths
					0.0f);
			g2d.setStroke(dashed);
			Random rand = new Random();
	
			for ( Line l : lines) {
				System.out.format("{(%f, %f),(%f, %f)}\n",l.x1,l.y1,l.x2,l.y2);
				Color color = new Color(rand.nextInt(0xFFFFFF)).brighter(); // make a random color and brighten it up
				g2d.setColor(color);
				g2d.drawLine((int)l.x1,(int)l.y1,(int)l.x2,(int)l.y2);
			}

			JLabel picLabel = new JLabel(new ImageIcon(myPicture));
			add(picLabel);

		} catch (IOException e) {
			System.err.println(e);
		}

		setSize(600,600);
        	setVisible(true);
	}

	public static void main(String [] args){
		new GUI();
	}
}
