package de.codesourcery.fancontrol;

import org.apache.commons.lang3.Validate;

import javax.swing.JPanel;
import javax.swing.ToolTipManager;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.geom.Rectangle2D;
import java.util.*;
import java.util.List;

public class FanSpeedMappingEditor extends JPanel
{
    private static final int SELECTION_RADIUS_IN_PIXELS = 10;

    private float scaleX,scaleY;
    private int originX,originY;

    private ThermalZone zone;
    private List<FanSpeedMapping> mapping = new ArrayList<>();

    private FanSpeedMapping dragged;
    private FanSpeedMapping highlighted;
    private Config config;

    public FanSpeedMappingEditor() {

        setBackground(Color.WHITE);
        ToolTipManager.sharedInstance().registerComponent(this);

        addMouseListener(new MouseAdapter()
        {
            @Override
            public void mouseClicked(MouseEvent e)
            {
                if ( dragged == null && e.getButton() == MouseEvent.BUTTON3 )
                {
                    if (  highlighted != null )
                    {
                        mapping.remove(highlighted);
                        highlighted = null;
                        repaint();
                    } else {
                        final Point p = viewToModel(e.getPoint());
                        mapping.add( new FanSpeedMapping(p.x,p.y) );
                        repaint();
                    }
                }
            }

            @Override
            public void mousePressed(MouseEvent e)
            {
                if ( e.getButton() == MouseEvent.BUTTON1 ) {
                    if ( highlighted != null ) {
                        dragged = highlighted;
                    }
                }
            }

            @Override
            public void mouseReleased(MouseEvent e)
            {
                if ( dragged != null ) {
                    dragged = null;
                }
            }
        });
        addMouseMotionListener(new MouseMotionAdapter()
        {
            @Override
            public void mouseDragged(MouseEvent e)
            {
                if ( dragged != null )
                {
                    final Point newLoc = viewToModel(e.getPoint());
                    dragged.setFromPoint(newLoc);
                    repaint();
                }
            }

            @Override
            public void mouseMoved(MouseEvent e)
            {
                final Point modelCoords = viewToModel(e.getPoint());
                final FanSpeedMapping closest = findClosest(modelCoords);
                if (closest != null)
                {
                    if (modelToView(closest.toPoint()).distance(e.getPoint()) < SELECTION_RADIUS_IN_PIXELS )
                    {
                        changeHighlight(closest);
                    } else {
                        changeHighlight(null);
                    }
                }
            }
        });
    }

    private void changeHighlight(FanSpeedMapping newHl)
    {
        if ( ! Objects.equals( this.highlighted, newHl ) ) {
            this.highlighted = newHl;
            if ( newHl != null ) {
                setToolTipText( newHl.temperature+" ° -> "+newHl.fanSpeed+" %" );
            } else {
                setToolTipText(null);
            }
            repaint();
        }
    }

    private FanSpeedMapping findClosest(Point modelCoords) {

        if ( mapping.isEmpty() ) {
            return null;
        }
        FanSpeedMapping p = mapping.get(0);
        FanSpeedMapping closest = p;
        double dMin = modelCoords.distance(p.toPoint());
        for ( int i = 1 ; i < mapping.size() ; i++ )
        {
            p = mapping.get(i);
            double d = modelCoords.distance(p.toPoint());
            if ( d < dMin ) {
                closest = p;
                dMin = d;
            }
        }
        return closest;
    }

    @Override
    protected void paintComponent(Graphics g)
    {
        super.paintComponent( g );

        final FontMetrics metrics = g.getFontMetrics();

        // calculate space needed left of the Y axis
        double yLabelMaxWidth=0;
        for ( int y = 0 ; y <= 100 ; y+= 10 )
        {
            final Rectangle2D rect = metrics.getStringBounds( Integer.toString( y ), g );
            if ( y == 0 || rect.getWidth() > yLabelMaxWidth ) {
                yLabelMaxWidth = rect.getWidth();
            }
        }
        final int yTickLen = 10;
        // space needed left of the Y axis line
        int leftSpace = (int) (yLabelMaxWidth + yTickLen/2 + 2);

        // calculate space needed below the X axis line
        final int xTickLen = 10;
        // calculate space needed left of the Y axis
        double xLabelMaxHeight = metrics.getStringBounds( Integer.toString( 100 ), g ).getHeight();

        // space needed below the X axis line
        int bottomSpace = (int) (xLabelMaxHeight + xTickLen/2 + 2);

        // render axis
        final int drawAreaWidth = (int) (getWidth()*0.9f);
        final int drawAreaHeight = (int) (getHeight()*0.8f);

        scaleX = drawAreaWidth  / 110.0f;
        scaleY = drawAreaHeight / 110.0f;

        originX = getWidth() - drawAreaWidth;
        originY = getHeight() - ( getHeight() - drawAreaHeight );

        // render X axis
        g.setColor( Color.BLACK);
        g.drawLine( originX, originY, originX + drawAreaWidth, originY);

        float xStep = drawAreaWidth / 11.0f;
        for ( int i=0 ; i <11 ; i++ ) {
            int x = (int) (originX + i * xStep);
            final int tickBottomY = originY + xTickLen / 2;
            g.drawLine(x, tickBottomY, x, originY - xTickLen/2 );
            g.drawString( Integer.toString(i*10), x , originY + bottomSpace );
        }

        // render Y axis
        g.drawLine( originX, originY, originX , originY - drawAreaHeight );
        float yStep = drawAreaHeight / 11.0f;
        for ( int i=0 ; i < 11 ; i++ ) {
            int y = (int) (originY - i*yStep);
            final int tickLeftX = originX - yTickLen / 2;
            g.drawLine(tickLeftX, y, tickLeftX + yTickLen, y );
            final String string = Integer.toString( i * 10 );
            g.drawString( string, originX - leftSpace  , y);
        }

        if ( mapping == null ) {
            return;
        }

        // draw curve
        this.mapping.sort(Comparator.comparingInt(a -> a.temperature));
        Point previousPoint = null;
        for ( FanSpeedMapping m : this.mapping ) {
            final Point pModel = new Point( m.temperature, m.fanSpeed );
            final Point pView = modelToView(pModel);

            g.setColor(Color.LIGHT_GRAY);
            g.drawLine(pView.x,pView.y,pView.x,originY);
            g.drawLine(pView.x,pView.y,originX,pView.y);

            g.setColor(Color.BLUE);
            if ( previousPoint != null ) {
                g.drawLine(previousPoint.x,previousPoint.y, pView.x, pView.y );
            }

            if ( Objects.equals( highlighted, m ) ) {
                g.setColor(Color.YELLOW);
            } else {
                g.setColor(Color.RED);
            }
            g.fillRect(pView.x-4,pView.y-4, 8, 8 );
            previousPoint = pView;
        }

        // draw min/max fan speed line
        g.setColor(Color.RED);
        for ( Point pModel : List.of(  new Point( 0, config.minFanSpeed ),  new Point( 0, config.maxFanSpeed ) ) )
        {
            final Point pView = modelToView(pModel);
            g.drawLine(originX+xTickLen/2,pView.y,originX + drawAreaWidth, pView.y );
        }
    }

    private Point modelToView(Point p) {
        int newX = (int) (originX + p.x * scaleX);
        int newY = (int) (originY - p.y * scaleY);
        return new Point(newX,newY);
    }

    private Point viewToModel(Point p)
    {
        final int dx = p.x - originX;
        final int dy = originY - p.y;

        int temp = (int) (dx / scaleX);
        int fanSpeed = (int) (dy / scaleY);
        temp = Math.max(0,Math.min(temp,100));
        fanSpeed = Math.max(0,Math.min(fanSpeed,100));
        return new Point(temp,fanSpeed);
    }

    public void saveMapping()
    {
        if ( this.zone != null ) {
            this.zone.setFanSpeedMapping(this.mapping);
        }
    }

    public void setThermalZone(Config config, ThermalZone zone)
    {
        Validate.notNull(zone, "zone must not be null");

        if ( this.zone != null && ! this.zone.equals( zone ) ) {
            saveMapping();
        }
        this.config = config;
        this.zone = zone;
        this.mapping = zone.getFanSpeedMapping();
        repaint();
    }
}